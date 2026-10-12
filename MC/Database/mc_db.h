/**
 * @file    mc_db.h
 * @brief   电机控制数据库（共享定义与运行数据）
 *
 * 统一存放控制相关的共享数据，供各层引用：
 *  - 模式枚举：Logic 状态机切换、Control 三环分发
 *  - 控制目标值：上位机/用户写入，控制环读取
 *  - 控制当前值：控制环采样/估计写入，外部读取
 *
 * 对外只暴露一个入口变量 mc（结构体 MC_Handle_t）：
 *  - mc.mode        当前运行模式（User/上位机写入，Control 分发读取）
 *  - mc.target      控制目标值（User/上位机写入，控制环读取）
 *  - mc.state       控制当前值（控制环采样写入，外部读取）
 *  - mc.device      运行设备句柄（Control_Init 初始化，各层读写）
 *  - mc.sched       分频调度状态（Control 三环分发使用）
 */
#ifndef MC_DB_H
#define MC_DB_H

#include <stdint.h>
#include "motor_pwm.h"      /* MotorPwm_Handle_t */
#include "current_sensor.h" /* CurrentSensor_Handle_t */
#include "pos_sensor.h"     /* PosSensor_Handle_t */
#include "mc_current.h"     /* CurrentLoop_t（电流环控制器） */
#include "mc_speed.h"       /* SpeedLoop_t（速度环控制器） */
#include "mc_enc_calib.h"   /* EncCalib_t（编码器校准：两点锁轴 / 慢速旋转） */

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 运行模式 ==================== */
typedef enum {
    MODE_STOP = 0,               /**< 停止：关断输出 */
    MODE_CURRENT_CALIB,          /**< 校准：电流偏置校准（上电第一段） */
    MODE_ENCODER_CALIB,          /**< 校准：编码器**两点锁轴**校准（同时定零点 offset 与方向 dir；
                                      不是"对齐 A 相"，单点定不了方向 —— 见 control.c 的说明） */
    MODE_FULL_OPEN_LOOP,         /**< 全开环：电压/频率开环，无反馈 */
    MODE_HALF_OPEN_LOOP,         /**< 半开环：角度闭环（编码器反馈）、电流开环（电压直接给定） */
    MODE_CURRENT_LOOP,           /**< 电流环：Id/Iq 闭环 */
    MODE_SPEED_CURRENT_LOOP,     /**< 速度电流环：速度环 + 电流环 */
    MODE_POS_SPEED_CURRENT_LOOP, /**< 位置速度电流环：位置环 + 速度环 + 电流环 */
    MODE_NUM,
} mode_t;

/**
 * @brief 获取模式名称（调试打印）
 */
const char *mc_db_mode_name(mode_t mode);

/* ==================== 控制目标值 ====================
 * 单位约定（全工程统一，改字段前先看这里）：
 *   电流   A（安培），由 current_sensor 的 (raw−bias)×A_PER_LSB 得到
 *   电压   调制比（无量纲），1.0 = 相电压峰值 Udc/2；**不是 V**，
 *          换算成伏特要乘实际母线电压（Udc 目前未测量，见 drv8311.h）
 *   角度   rad（机械/电角度都在 mc_state 里注明）
 *   速度   rpm（对外） / rps（pos_sensor 内部）
 *   频率   Hz
 */
typedef struct {
    float pos_target;   /**< 位置目标 (rad) */
    float spd_target;   /**< 速度目标 (rpm)：速度环（MODE_SPEED_CURRENT_LOOP）用 */
    float iq_max;       /**< **用户限幅** (A)：速度/位置模式下用户指定的 Iq 给定上限
                             （＝转矩限幅）。速度环每拍按 min(用户限幅, 保底限幅
                             CURRENT_I_MAX_GUARD) 限幅 —— 保底限幅不可被它顶破；
                             上电默认 = CURRENT_I_MAX_USER */
    float iq_target;    /**< Iq 电流目标 (A)：**仅电流环模式**（MODE_CURRENT_LOOP）的用户给定；
                             速度/位置模式下电流给定由外环写 state.iq_ref，本字段不参与 */
    float id_target;    /**< Id 电流目标 (A)；仅电流环用，半开环的 Ud 请用 ud_target */
    float ud_target;    /**< 半开环：d 轴电压给定（调制比 [-1,1]，1 = Udc/2） */
    float uq_target;    /**< 半开环：q 轴电压给定（调制比 [-1,1]，1 = Udc/2） */
    float ol_volt;      /**< 全开环：q 轴电压幅值（调制比 0~1，1 = Udc/2） */
    float ol_freq;      /**< 全开环：电频率 (Hz) */
} mc_target_t;

/* ==================== 控制当前值 ====================
 * 写入情况（谁填的）：
 *   ol_angle / id / iq / ud / uq —— 控制环每拍写（Foc_ObserveDq / 各 Mode_xxx）
 *   id_ref / iq_ref —— **当前生效的 dq 电流给定**（外环输出→内环给定）：
 *        电流环模式 = target.id_target / target.iq_target（高频环每拍搬一次）；
 *        速度/位置模式 = 速度环输出（中频环每拍写）、id_ref 恒 0
 *   其余字段标注 [预留] 的目前没人填：要用请直接读设备句柄
 *   （mc.device.pos 的 PosSensor_GetXxx / mc.device.cur 的 CurrentSensor_GetXxx），
 *   避免同一份数据两处搬运、两处不一致
 */
typedef struct {
    float mech_angle;   /**< [预留] 机械角度 (rad)：用 PosSensor_GetMechAngleMotor() */
    float elec_angle;   /**< [预留] 电角度 (rad)：用 PosSensor_GetElecAngle() */
    float speed_rpm;    /**< [预留] 转速 (rpm)：用 PosSensor_GetSpeedRpm() */
    float position;     /**< [预留] 位置 (rad)：用 PosSensor_GetPosition() */
    float ia, ib, ic;   /**< [预留] 三相电流 (A)：用 CurrentSensor_GetIa/Ib/Ic() */
    float ol_angle;     /**< 全开环：开环电角度 (rad)，每高频拍积分 */
    float id, iq;       /**< dq 轴电流反馈 (A)：电流环/半开环/全开环观测共用 */
    float id_ref;       /**< d 轴电流给定 (A)：送进电流环的值（已限幅），通常 0 */
    float iq_ref;       /**< q 轴电流给定 (A)：送进电流环的值（已限幅）= 转矩指令 */
    float ud, uq;       /**< dq 轴电压（调制比 [-1,1]，1 = Udc/2；非 V） */
} mc_state_t;

/* ==================== 运行设备（句柄统一封装） ==================== */
typedef struct {
    MotorPwm_Handle_t      pwm;    /**< 电机 PWM 设备句柄 */
    CurrentSensor_Handle_t cur;    /**< 三相电流传感器设备句柄 */
    PosSensor_Handle_t     pos;    /**< 位置传感器设备句柄 */
} mc_device_t;

/* ==================== 控制环 ====================
 * 级联：位置环（5kHz，待实现）→ 速度环（10kHz）→ 电流环（20kHz）
 * 电流环：mc_current 模块（dq PI + 前馈解耦 + 矢量限幅 + 抗饱和），
 *         增益/限幅/前馈参数都可在运行中直接改 mc.current.xxx 生效
 * 速度环：mc_speed 模块（PI + 给定斜坡 + 输出限幅 → Iq 给定），
 *         增益/限幅/斜坡可在运行中直接改 mc.speed.pid.kp/ki、mc.speed.iq_max、
 *         mc.speed.ramp_rpm_s 生效（pid 内核见 Algorithm/pid.c）
 * 待实现：位置环（可复用 Algorithm/pid.c，输出 → spd_target）
 */

/* ==================== 调度状态（分频调度器） ==================== */
typedef struct {
    uint16_t mid_cnt;    /**< 中频环分频计数（2 分频 = 10kHz） */
    uint16_t low_cnt;    /**< 低频环分频计数（4 分频 = 5kHz） */
} mc_sched_t;

/* ==================== 控制数据库（统一封装） ==================== */
/**
 * @brief 电机控制数据库结构：模式 + 目标值 + 当前值 + 设备句柄 + 调度状态 统一封装
 */
typedef struct {
    mode_t      mode;    /**< 当前运行模式 */
    mc_target_t target;  /**< 控制目标值 */
    mc_state_t  state;   /**< 控制当前值 */
    mc_device_t device;  /**< 运行设备句柄（PWM/电流/位置，Control_Init 初始化） */
    mc_sched_t  sched;   /**< 分频调度状态（Control 三环分发使用） */
    EncCalib_t  enc_calib;  /**< 编码器校准算法句柄（状态机+诊断量；算法在 Algorithm/mc_enc_calib.c） */
    CurrentLoop_t  current;    /**< 电流环控制器（Control_Init 初始化，运行中可直接改字段） */
    SpeedLoop_t    speed;      /**< 速度环控制器（Control_Init 初始化，运行中可直接改字段） */
    uint8_t     enc_calib_done;  /**< 编码器校准状态：1 = 已有有效偏移+方向（本次上电校准完成）；掉电保持待开发 */
    uint8_t     calib_report;    /**< 编码器校准结果一次性打印标志：主循环打印后清 0 */
    uint8_t     oc_latched;      /**< 过流保护锁定：1 = 已触发过流（电流环），测试入口不再自动切入 */
    uint8_t     fault_report;    /**< 故障一次性打印标志：主循环打印后清 0 */
    uint16_t    oc_cnt;          /**< 过流连续超阈值计数（未超时清零） */

    /* ---- 驱动器故障（nFAULT）诊断：Control 层填，供上报 ----
     * 用途：区分"常驻故障"（一直不恢复 ⇒ 查硬件）与"间歇故障"（拉了又放 ⇒ 优先怀疑
     * 供电跌落/接插件接触/过流重试），并给出故障持续了多久 —— 这两者排查方向不同 */
    uint16_t    drv_fault_ticks; /**< 本次上电累计 nFAULT 拉低的拍数（20kHz ⇒ 20000 = 1s；饱和 65535） */
    uint16_t    drv_fault_cnt;   /**< nFAULT 拉低次数 */
    uint8_t     drv_fault_seen;  /**< 内部边沿标志：故障期间还没做过"复位校准" */
} MC_Handle_t;

/* 全局共享实例（单电机系统） */
extern MC_Handle_t mc;

#ifdef __cplusplus
}
#endif

#endif /* MC_DB_H */
