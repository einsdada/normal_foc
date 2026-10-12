#ifndef __POS_SENSOR_H__
#define __POS_SENSOR_H__

#include "stdint.h"

/* 角度单位常量：全工程角度一律 rad（每计数弧度 = 2π/每圈计数） */
#ifndef TWO_PI
#define TWO_PI              6.283185307179586f   /* 2π (rad/圈) */
#endif

#define SSI_RAD_PER_COUNT   (TWO_PI / (float)MT6701_SSI_CPR)    /* SSI 每计数弧度 */
#define ABZ_RAD_PER_COUNT   (TWO_PI / (float)MT6701_ABZ_CPR)    /* ABZ 每计数弧度 */

/**
 * @brief  位置传感器类型
 * @note   位置传感器抽象层：SSI/ABZ 已实现，霍尔预留
 */
typedef enum {
    POS_SENSOR_SSI      = 0,   // SSI 绝对编码器（BSP 层 mt6701_ssi）
    POS_SENSOR_ABZ      = 1,   // ABZ 增量编码器（BSP 层 mt6701_abz）
    POS_SENSOR_HALL     = 2,   // 霍尔传感器（预留）
} PosSensor_Type_e;

/**
 * @brief  位置传感器句柄
 * @note   单位统一：角度/位置 rad，转速 rps（转/秒），rpm 由接口换算
 * @note   坐标系约定（elec_dir 作用范围）：
 *           传感器坐标系（原始）：mech_angle、mech_angle_last
 *           电机坐标系（×elec_dir，正方向 = 电角度增大方向 = 相序方向）：
 *             elec_angle、mech_speed(_filt)、position
 *         方向反的编码器（elec_dir=-1）下，原始机械角与电角度必然反向，
 *         这是磁铁安装方向决定的正常现象，不是 bug
 */
typedef struct {
    PosSensor_Type_e type;         // 传感器类型
    uint16_t         pole_pairs;   // 电机极对数（机械角度换算电角度用）
    float            mech_angle;   // 机械角度（rad，编码器原始值，传感器坐标系）
    float            mech_angle_last;  // 上次机械角度（原始值，解算速度/绝对位置用）
    float            elec_angle;   // 电角度（rad，= dir×机械×极对数 + offset，电机坐标系）
    float            mech_speed;   // 机械转速（rps，电机坐标系：正 = 电角度增大方向，原始差分值）
    float            mech_speed_filt; // 机械转速（rps，一阶低通滤波后，对外读取用，电机坐标系）
    float            elec_speed_filt; // 电角速度（rad/s）：UpdateDerived 里由 mech_speed_filt×2π×极对数
                                      //   算好缓存，20kHz 控制环只读（不在 ISR 里做浮点乘法）
    float            position;     // 绝对位置（rad，含整圈，电机坐标系，反转时为负）
    int8_t           elec_dir;     // 机械角→电角度方向（+1/-1，两点锁轴校准测得：
                                   //   电角度 = NormRad(elec_dir×机械角度×极对数 + elec_offset)）
    float            elec_offset;  // 电角度偏移（rad，编码器校准时写入）
    uint8_t          valid;        // 最新一帧是否有效
    /* ---- 帧新鲜度诊断（识别"角度冻结"，见 PosSensor_IsStale） ---- */
    uint32_t         frame_ok;     // 累计收到的**新**有效帧数
    uint16_t         miss;         // 连续多少次调用没有新帧
    uint16_t         miss_max;     // 本次上电以来 miss 的最大值（≥阈值 ⇒ 曾冻结过）
    uint8_t          stale;        // 1 = 编码器数据已失效（角度冻结）
    /* ---- 编码器帧自带的磁场诊断（SSI 帧 [9:6] 位，设备层解析） ----
     这两项能直接判断"角度不动"是不是**磁铁**的问题：
       mag_loss=1 或 mag_status≠0 ⇒ 磁铁丢失/太远/太近 ⇒ 角度输出冻结（CRC 仍正确）
       两者都正常而角度不动 ⇒ 没有重新锁存（查 CSN/SPI 时序） */
    uint8_t          mag_status;   // 磁场状态：0=正常 1=过强 2=过弱 3=无定义
    uint8_t          mag_loss;     // 1 = 编码器报告失磁（loss of track）
} PosSensor_Handle_t;

/* 连续多少次"没有新帧"就判失效：本函数由中频环 10kHz 调用 ⇒ 200 次 = 20ms
   （一帧只需 ~2.3us@10MHz，20ms 收不到一帧只能是链路断了） */
#define POS_SENSOR_STALE_CALLS   200u

/**
 * @brief  初始化位置传感器
 * @param  handle       句柄
 * @param  type         传感器类型
 * @param  pole_pairs   电机极对数（≥1）
 * @note   每圈计数由各 BSP 层宏定义（MT6701_SSI_CPR / MT6701_ABZ_CPR）
 */
void PosSensor_Init(PosSensor_Handle_t *handle, PosSensor_Type_e type, uint16_t pole_pairs);

/**
 * @brief  更新一次角度/速度/位置数据
 * @param  handle  句柄
 * @param  freq    本函数调用频率（Hz，如 1ms 调用 = 1000），用于解算速度
 * @retval 1=读到有效数据，0=暂无有效数据
 * @note   需要连续读取时在循环里以固定周期反复调用
 */
uint8_t PosSensor_Update(PosSensor_Handle_t *handle, uint16_t freq);

/**
 * @brief  获取机械角度（rad）——编码器**原始值**（传感器坐标系，不做方向处理）
 * @note   两点锁轴校准（算 θmech0 / Δθmech）必须用本原始值；
 *         编码器方向可能与电角度相反（elec_dir=-1），此时本角与电角度反向是正常的
 */
float PosSensor_GetMechAngle(PosSensor_Handle_t *handle);

/**
 * @brief  获取机械角度（rad）——**电机坐标系**（已乘 elec_dir，与电角度同向）
 * @note   与 GetSpeed/GetSpeedRpm/GetPosition 同坐标系，供显示/对轴使用
 */
float PosSensor_GetMechAngleMotor(PosSensor_Handle_t *handle);

/**
 * @brief  获取电角度（rad）
 * @note   已取模到 [0, 2π)
 */
float PosSensor_GetElecAngle(PosSensor_Handle_t *handle);

/**
 * @brief  获取机械转速（机械角速度，rps，转/秒）
 */
float PosSensor_GetSpeed(PosSensor_Handle_t *handle);

/**
 * @brief  获取电角速度（rad/s）——机械转速 × 2π × 极对数
 * @note   极对数由 PosSensor_Init 配置，换算在设备层完成；
 *         控制层做前馈解耦与角度延迟补偿时应直接用它，不要自己乘 2π×极对数
 */
float PosSensor_GetElecSpeed(PosSensor_Handle_t *handle);

/**
 * @brief  获取机械转速（rpm，转/分）= rps × 60
 */
float PosSensor_GetSpeedRpm(PosSensor_Handle_t *handle);

/**
 * @brief  设置电角度偏移（rad）
 * @param  handle       句柄
 * @param  offset_rad   偏移角（编码器校准时写入，见 elec_offset 注释）
 * @note   编码器零点校准：转子对齐 A 相磁轴（电角度约定 0）后调用
 *         PosSensor_SetElecOffset(handle, NormRad(-dir×θ_align×极对数))，
 *         使对齐位置的电角度归零；之后 FOC 电角度与转子磁极正确对应
 */
void PosSensor_SetElecOffset(PosSensor_Handle_t *handle, float offset_rad);

/**
 * @brief  设置机械角→电角度方向（两点锁轴校准测得）
 * @param  handle  句柄
 * @param  dir     +1 = 编码器计数方向与电机相序一致（默认）；-1 = 相反
 * @note   单点锁轴只能定"零点"，方向必须由第二点（矢量再锁到电角度 +90°，
 *         看机械角往哪边走）判定：
 *           θe = dir·极对数·θmech + offset
 *         方向反时电角度随转子反向变化，角度闭环（半开环/电流环）变成正反馈——
 *         电压矢量相对转子倒转、转矩角随即反向 → 平均转矩为 0 → 电机原地不转
 *         （只抖不动、电流全落在 d 轴上），与电压大小无关，加电压也修不好
 */
void PosSensor_SetElecDir(PosSensor_Handle_t *handle, int8_t dir);

/**
 * @brief  获取绝对位置（rad，含整圈，反转时为负）
 */
float PosSensor_GetPosition(PosSensor_Handle_t *handle);

/**
 * @brief  编码器数据是否已失效（角度冻结）
 * @param  handle  句柄
 * @retval 1 = 连续 ≥POS_SENSOR_STALE_CALLS 次调用都没有收到新的有效帧 ⇒ 角度读数冻结；
 *             此时"角度闭环"看到的是一个静止的转子，磁场不会旋转 ⇒ 无转矩、却一直有电流
 * @note   控制层用它做"新鲜度看门狗"：失效时把电流给定归零；报告用它定位故障
 *         （SSI 连续 CRC 失败、DMA 卡死、接线松动都会走到这里）
 */
uint8_t PosSensor_IsStale(const PosSensor_Handle_t *handle);

#endif
