#include "pos_sensor.h"
#include "mt6701.h"         /* BSP 层：MT6701 磁编码器 */

#define PI                  3.14159265f /* 圆周率 π（TWO_PI 已由头文件提供，角度单位 rad） */

/* 固定常量运算用括号包起来，编译期即折叠为单个常量，运行时无除法 */
#define INV_TWO_PI          (1.0f / TWO_PI)                     /* 1/2π ≈ 0.159155 */

/* 速度一阶低通滤波系数：y += α·(x−y)，α = 2π·fc/fs
   采样 fs = 10kHz（中频环），fc = 95Hz → α ≈ 0.06（时间常数 ≈1.7ms） */
#define SPEED_LPF_ALPHA     0.06f


/**
 * @brief  角度取模到 [0, 2π)
 */
static float NormRad(float rad)
{
    rad -= (float)(int)(rad * INV_TWO_PI) * TWO_PI;   /* 取模用乘倒数，无除法 */
    if (rad < 0.0f)
        rad += TWO_PI;
    return rad;
}

/**
 * @brief  角度差（rad）：处理 ±π 过零，返回有符号角度差
 */
static float AngleDelta(float new_rad, float old_rad)
{
    float d = new_rad - old_rad;
    if (d > PI)
        d -= TWO_PI;
    else if (d <= -PI)
        d += TWO_PI;
    return d;
}

/**
 * @brief  初始化位置传感器
 */
void PosSensor_Init(PosSensor_Handle_t *handle, PosSensor_Type_e type, uint16_t pole_pairs)
{
    handle->type            = type;
    handle->pole_pairs      = pole_pairs;
    handle->mech_angle      = 0.0f;
    handle->mech_angle_last = 0.0f;
    handle->elec_angle      = 0.0f;
    handle->mech_speed      = 0.0f;
    handle->mech_speed_filt = 0.0f;
    handle->elec_speed_filt = 0.0f;
    handle->position        = 0.0f;
    handle->elec_dir        = 1;        /* 方向默认 +1：两点锁轴校准后按测得结果改写 */
    handle->elec_offset     = 0.0f;
    handle->valid           = 0;
    handle->frame_ok        = 0u;
    handle->miss            = 0u;
    handle->miss_max        = 0u;
    handle->stale           = 0u;
    handle->mag_status      = 0u;
    handle->mag_loss        = 0u;

    switch (type)
    {
    case POS_SENSOR_SSI:
        MT6701_Ssi_Init();                  /* 启动 SSI 编码器硬件 */
        break;
    case POS_SENSOR_ABZ:
        MT6701_Abz_Init();                  /* 启动 ABZ 编码器计数 */
        break;
    default:
        break;                              /* 预留类型暂不初始化 */
    }
}

/**
 * @brief  电角度 + 速度 + 位置解算（SSI/ABZ 共用；调用前 handle->mech_angle 已更新）
 * @param  handle  句柄
 * @param  freq    本函数调用频率（Hz，解算速度用）
 * @note   电角度 = dir×机械×极对数 + offset，取模 [0,2π)；
 *         速度/位置用**电机坐标系**增量（乘 elec_dir）：正方向 = 电角度增大方向，
 *         这样电机正转时 rpm>0、position 递增，与电角度/相序同向；
 *         原始编码器机械角 mech_angle 不做方向处理（传感器真值，方向由磁铁安装决定）
 */
static void PosSensor_UpdateDerived(PosSensor_Handle_t *handle, uint16_t freq)
{
    handle->elec_angle = NormRad((float)handle->elec_dir * handle->mech_angle
                                 * (float)handle->pole_pairs
                                 + handle->elec_offset);

    if (handle->valid)                      /* 已有上一帧才解算速度/位置 */
    {
        float delta  = AngleDelta(handle->mech_angle, handle->mech_angle_last);
        float ddelta = (float)handle->elec_dir * delta;    /* 电机坐标系增量 */

        handle->mech_speed = ddelta * INV_TWO_PI * (float)freq;   /* 原始 rps，无除法 */
        handle->mech_speed_filt += SPEED_LPF_ALPHA
                                   * (handle->mech_speed - handle->mech_speed_filt);  /* 一阶低通 */
        /* 电角速度（rad/s）＝ 机械转速 × 2π × 极对数：换算放在 10kHz 更新里算一次，
           20kHz 控制环只读字段（前馈解耦/角度延迟补偿要用），ISR 里不做浮点乘法。
           一个滤波状态、多种单位表示：机械 rpm/rps 与电角速度都由 mech_speed_filt 派生 */
        handle->elec_speed_filt = handle->mech_speed_filt * TWO_PI * (float)handle->pole_pairs;
        handle->position  += ddelta;                        /* rad（电机坐标系，反转时为负） */
    }
    handle->mech_angle_last = handle->mech_angle;
    handle->valid           = 1;
}

/**
 * @brief  更新一次角度/速度/位置数据，返回是否有有效数据
 * @note   只做"读原始角度 → 机械角"，其余量统一由 PosSensor_UpdateDerived 解算
 */
uint8_t PosSensor_Update(PosSensor_Handle_t *handle, uint16_t freq)
{
    Encoder_RawData_t data;

    switch (handle->type)
    {
    case POS_SENSOR_SSI:
        /* 帧新鲜度统计：TakeNewFrame 读后清零 ⇒ 能区分"新帧"与"卡住的旧帧"。
           没有新帧就累计 miss，超阈值置 stale（角度冻结）——
           这是"电机不转但一直有电流"的常见根因，见 PosSensor_IsStale 的说明 */
        if (MT6701_Ssi_TakeNewFrame() != 0u)
        {
            handle->frame_ok++;
            handle->miss = 0u;
        }
        else if (handle->miss < 0xFFFFu)
        {
            handle->miss++;
            if (handle->miss > handle->miss_max) handle->miss_max = handle->miss;
        }
        handle->stale = (uint8_t)((handle->miss >= POS_SENSOR_STALE_CALLS) ? 1u : 0u);

        /* SSI：14bit 绝对角度 → 机械角（rad） */
        if (MT6701_Ssi_Process_RawData(&data))      /* BSP 层读到有效帧 */
        {
            handle->mech_angle = (float)data.angle * SSI_RAD_PER_COUNT;   /* 1 次乘法 */
            /* 帧自带的磁场诊断：磁铁丢失/太远/太近 ⇒ 角度会冻结（而 CRC 仍正确） */
            handle->mag_status = (uint8_t)data.field_strength;
            handle->mag_loss   = data.loss_of_track;
            PosSensor_UpdateDerived(handle, freq);
        }
        break;

    case POS_SENSOR_ABZ:
        /* ABZ：增量计数取模归一化 → 机械角（rad） */
        {
            int32_t m = MT6701_Abz_GetCount() % MT6701_ABZ_CPR;
            if (m < 0) m += MT6701_ABZ_CPR;
            handle->mech_angle = (float)m * ABZ_RAD_PER_COUNT;            /* 1 次乘法 */
            PosSensor_UpdateDerived(handle, freq);
        }
        handle->stale = 0u;         /* ABZ 不走 SSI 帧新鲜度判据 */
        break;

    default:
        handle->valid       = 0;                    /* 预留类型暂无数据 */
        handle->stale       = 0u;
        break;
    }
    return handle->valid;
}

uint8_t PosSensor_IsStale(const PosSensor_Handle_t *handle)
{
    return handle->stale;
}

/**
 * @brief  获取机械角度（rad）——**编码器原始值**（传感器坐标系）
 * @note   不做方向处理：编码器计数方向由磁铁安装决定，可能与电角度方向相反
 *         （此时 elec_dir=-1，本函数返回的角与电角度反向，属正常现象）。
 *         要看与电角度同向的机械角请用 PosSensor_GetMechAngleMotor()。
 *         编码器校准（两点锁轴）必须用本原始值算 θmech0/Δθmech
 */
float PosSensor_GetMechAngle(PosSensor_Handle_t *handle)
{
    return handle->mech_angle;
}

/**
 * @brief  获取机械角度（rad）——**电机坐标系**（已乘 elec_dir）
 * @note   正方向与电角度/相序一致：电机正转时本角与电角度同向增大。
 *         与 GetSpeed/GetSpeedRpm/GetPosition 同一坐标系，用于对轴/显示
 */
float PosSensor_GetMechAngleMotor(PosSensor_Handle_t *handle)
{
    return NormRad((float)handle->elec_dir * handle->mech_angle);
}

/**
 * @brief  获取电角度（rad）
 */
float PosSensor_GetElecAngle(PosSensor_Handle_t *handle)
{
    return handle->elec_angle;
}

/**
 * @brief  获取机械转速（rps，转/秒）
 * @note   返回一阶低通滤波后的速度（原始差分值噪声大）
 */
float PosSensor_GetSpeed(PosSensor_Handle_t *handle)
{
    return handle->mech_speed_filt;
}

/**
 * @brief  设置电角度偏移（rad）——编码器零点校准结果
 */
void PosSensor_SetElecOffset(PosSensor_Handle_t *handle, float offset_rad)
{
    handle->elec_offset = offset_rad;
}

/**
 * @brief  设置机械角→电角度方向（两点锁轴校准结果）
 * @note   非 0 一律归一为 ±1；调用方负责保证与 elec_offset 出自同一次校准
 */
void PosSensor_SetElecDir(PosSensor_Handle_t *handle, int8_t dir)
{
    handle->elec_dir = (dir < 0) ? (int8_t)-1 : (int8_t)1;
}

/**
 * @brief  获取电角速度（rad/s）= 机械转速 × 2π × 极对数
 * @note   极对数是本传感器的配置量（PosSensor_Init 传入），换算在 PosSensor_UpdateDerived
 *         里按 10kHz 更新频率算好缓存（elec_speed_filt），本函数只读，20kHz 控制环
 *         取用时不含浮点乘法；
 *         控制层做前馈解耦/角度延迟补偿直接取用，不必自己乘 2π 和极对数
 *         （否则同一换算在多处重复、单位也不明确）；
 *         符号与 GetSpeed 一致：正 = 电角度增大方向（电机坐标系）
 */
float PosSensor_GetElecSpeed(PosSensor_Handle_t *handle)
{
    return handle->elec_speed_filt;
}

/**
 * @brief  获取机械转速（rpm，转/分）= rps × 60
 * @note   返回滤波后转速（同 GetSpeed）
 */
float PosSensor_GetSpeedRpm(PosSensor_Handle_t *handle)
{
    return handle->mech_speed_filt * 60.0f;
}

/**
 * @brief  获取绝对位置（rad）
 */
float PosSensor_GetPosition(PosSensor_Handle_t *handle)
{
    return handle->position;
}
