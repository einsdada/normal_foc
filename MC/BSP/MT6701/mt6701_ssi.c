#include "mt6701.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi3;

/* 寄存器级 GPIO 操作：BSRR 原子置位/复位，省去 HAL 函数调用
   低 16 位写 1 = 置高，高 16 位写 1 = 置低 */
#define SSI_CSN_LOW()    (MT6701_SSI_CSN_PORT->BSRR = (uint32_t)MT6701_SSI_CSN_PIN << 16U)
#define SSI_CSN_HIGH()   (MT6701_SSI_CSN_PORT->BSRR = (uint32_t)MT6701_SSI_CSN_PIN)

static uint8_t rx_buf[3];

/* 连续多少次调用没等到 DMA 完成就判定"完成中断丢了/传输卡死"：
   本函数由中频环 10kHz 调用 → 200 次 = 20ms（帧本身只要 ~2.3us @10MHz） */
#define SSI_BUSY_TIMEOUT   200u

/* DMA 读取状态 */
static volatile uint16_t enc_angle = 0;   /* 最近一次 CRC 校验通过的角度 */
static volatile uint32_t enc_raw   = 0;   /* 最近一帧 CRC 通过的**原始 24bit**（含磁场状态位，诊断用） */
static volatile uint8_t  enc_valid = 0;   /* 是否已经读到过有效帧 */
static volatile uint8_t  enc_busy  = 0;   /* DMA 是否正在搬运 */
static uint16_t          enc_busy_cnt = 0;/* 连续 busy 次数（卡死兜底计时） */
static volatile uint8_t  enc_new_frame = 0; /* 有"新"有效帧待取走（TakeNewFrame 读后清零） */


void MT6701_Ssi_Init(void)
{
    /* MODE 置高选择 SSI 模式（CSN 由软件控制，先拉高保持空闲） */
    MT6701_SSI_MODE_PORT->BSRR = (uint32_t)MT6701_SSI_MODE_PIN;
    SSI_CSN_HIGH();

    enc_angle     = 0;
    enc_raw       = 0;
    enc_valid     = 0;
    enc_busy      = 0;
    enc_busy_cnt  = 0;
    enc_new_frame = 0;
}


uint8_t MT6701_Ssi_TakeNewFrame(void)
{
    if (enc_new_frame != 0u)
    {
        enc_new_frame = 0u;
        return 1u;
    }
    return 0u;
}


/* ---- CRC-6（多项式 0x03 = x^6 + x + 1，初值 0，18 位数据 MSB 先出）查表实现 ----
 * 原实现是逐位循环（18 次迭代，且跑在最高优先级的 DMA 完成中断里）：
 *      crc=0; 每 bit:{ msb=crc[5]; crc<<=1; if(msb^bit) crc^=0x03; crc&=0x3F; }
 * 该 LFSR 对输入是线性的、初值为 0 ⇒ 把 18 位拆成 3 个 6 位块后
 *      CRC(data) = T1[a] ^ T2[b] ^ T3[c]
 * 于是 18 次循环 → 3 次查表 + 2 次异或（表共 192 字节）。
 * 生成与验证：对"线性基"（每块单独遍历）以及 2000 组随机 18 位输入，
 * 与逐位实现结果完全一致（0 处不一致）。 */
static const uint8_t crc6_tbl[3][64] = {
    {   /* T1[a] = CRC(a << 12) */
        0x00, 0x0F, 0x1E, 0x11, 0x3C, 0x33, 0x22, 0x2D,
        0x3B, 0x34, 0x25, 0x2A, 0x07, 0x08, 0x19, 0x16,
        0x35, 0x3A, 0x2B, 0x24, 0x09, 0x06, 0x17, 0x18,
        0x0E, 0x01, 0x10, 0x1F, 0x32, 0x3D, 0x2C, 0x23,
        0x29, 0x26, 0x37, 0x38, 0x15, 0x1A, 0x0B, 0x04,
        0x12, 0x1D, 0x0C, 0x03, 0x2E, 0x21, 0x30, 0x3F,
        0x1C, 0x13, 0x02, 0x0D, 0x20, 0x2F, 0x3E, 0x31,
        0x27, 0x28, 0x39, 0x36, 0x1B, 0x14, 0x05, 0x0A,
    },
    {   /* T2[b] = CRC(b << 6) */
        0x00, 0x05, 0x0A, 0x0F, 0x14, 0x11, 0x1E, 0x1B,
        0x28, 0x2D, 0x22, 0x27, 0x3C, 0x39, 0x36, 0x33,
        0x13, 0x16, 0x19, 0x1C, 0x07, 0x02, 0x0D, 0x08,
        0x3B, 0x3E, 0x31, 0x34, 0x2F, 0x2A, 0x25, 0x20,
        0x26, 0x23, 0x2C, 0x29, 0x32, 0x37, 0x38, 0x3D,
        0x0E, 0x0B, 0x04, 0x01, 0x1A, 0x1F, 0x10, 0x15,
        0x35, 0x30, 0x3F, 0x3A, 0x21, 0x24, 0x2B, 0x2E,
        0x1D, 0x18, 0x17, 0x12, 0x09, 0x0C, 0x03, 0x06,
    },
    {   /* T3[c] = CRC(c) */
        0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
        0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
        0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
        0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
        0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
        0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
        0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
        0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02,
    },
};

static uint8_t CalcCRC(uint32_t data18)
{
    return (uint8_t)(crc6_tbl[0][(data18 >> 12) & 0x3Fu]
                   ^ crc6_tbl[1][(data18 >>  6) & 0x3Fu]
                   ^ crc6_tbl[2][(data18      ) & 0x3Fu]);
}


/* DMA 搬完 3 字节后由 DMA1_Channel2 中断进入。
   HAL 在回调前已关闭 SPI 并清空 FIFO，所以这里拉高 CSN 是安全的 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3) return;

    SSI_CSN_HIGH();     // 一帧结束，释放片选
    enc_busy     = 0;
    enc_busy_cnt = 0;

    uint32_t raw24 = ((uint32_t)rx_buf[0] << 16) |
                     ((uint32_t)rx_buf[1] << 8)  |
                     (uint32_t)rx_buf[2];

    /* ⚠⚠ 全 0 帧必须**显式拒绝** —— 这是本驱动最容易骗过自己的地方：
       CRC-6（LFSR 初值 0）对全 0 输入的输出就是 0，所以 raw24 == 0 时
       (raw24 & 0x3F) == CalcCRC(0) == 0 **校验会通过**！
       一旦 MISO(PC11) 断线/虚焊/编码器没驱动，收到的永远是 0x000000：
         · 固件把"恒为 0 的角度"当成有效帧 ⇒ 角度读数与轴的转动彻底脱钩
         · 帧却一直"有效"（新帧计数照涨）⇒ 新鲜度看门狗也不会报
         · 现象：校准 dm=0、闭环角度不更新、电机不转但一直有电流
       全 1 帧同理挡掉（它的 CRC 字段对不上，本就来通不过，写明更清楚）。
       拒绝后不置 enc_new_frame ⇒ 设备层的 miss 计数会涨 ⇒ 20ms 后判"编码器失效"
       并打出 FAULT: encoder stale —— 把静默故障变成明确报警 */
    if ((raw24 == 0u) || (raw24 == 0xFFFFFFu))
    {
        return;
    }

    if ((raw24 & 0x3F) == CalcCRC((raw24 >> 6) & 0x3FFFF))
    {
        enc_angle     = (raw24 >> 10) & 0x3FFF;     // 校验通过才更新
        enc_raw       = raw24;                      // 整帧留一份：磁场状态/失磁位在 [9:6]
        enc_valid     = 1;
        enc_new_frame = 1;                          // 标记"有新帧"（设备层统计新鲜度用）
    }
}


uint8_t MT6701_Ssi_Process_RawData(Encoder_RawData_t *data)
{
    // 1. 先把下一帧发出去（非阻塞，立即返回；CSN 提前拉低保证建立时间 TL>=100ns）
    if (enc_busy)
    {
        /* 卡死兜底：DMA 完成中断若丢失，enc_busy 永远为 1 → 角度永久冻结，
           电机角度看就是"转子不转/角度不动"，且校准状态机的"位置稳定"判据
           还会误判为收敛成功。超时后中止本次传输、复位状态重新起帧。 */
        if (++enc_busy_cnt >= SSI_BUSY_TIMEOUT)
        {
            enc_busy_cnt = 0u;
            enc_busy     = 0u;
            SSI_CSN_HIGH();
            if (hspi3.State != HAL_SPI_STATE_READY)
            {
                HAL_SPI_Abort(&hspi3);      /* 半途卡住的 SPI/DMA：中止后重新起帧 */
            }
        }
    }
    if (!enc_busy)
    {
        SSI_CSN_LOW();
        enc_busy = 1;
        if (HAL_SPI_Receive_DMA(&hspi3, rx_buf, 3) != HAL_OK)
        {
            enc_busy = 0;
            SSI_CSN_HIGH();     // 启动失败，别把 CSN 留在低电平
        }
    }

    // 2. 返回上一帧的结果（第一次调用时还没有数据，返回 0）
    if (!enc_valid) return 0;

    /* 整帧解析：18 位数据 = [23:10] 角度(14) + [9:8] 磁场状态 + [7] 按键 + [6] 失磁标志，
       [5:0] 是 CRC-6（见 CalcCRC 的校验式）。原来只取了角度，把**磁场状态与失磁标志
       整块丢掉了** —— 而"角度读数冻结但帧仍有效"恰恰要靠这两项来判断：
         · 磁铁掉了/离太远 ⇒ 磁场状态异常 + loss_of_track=1 ⇒ 角度输出冻结（CRC 仍正确）
         · 若这两项都正常而角度不动 ⇒ 说明没有重新锁存（查 CSN/SPI 时序） */
    data->angle          = enc_angle;
    data->field_strength = (Magnet_FieldStatus_e)((enc_raw >> 8) & 0x3u);
    data->push_button    = (uint8_t)((enc_raw >> 7) & 0x1u);
    data->loss_of_track  = (uint8_t)((enc_raw >> 6) & 0x1u);
    return 1;
}
