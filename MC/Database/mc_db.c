/**
 * @file    mc_db.c
 * @brief   电机控制数据库实现：全局共享实例 mc 定义 + 模式名称
 */
#include "mc_db.h"

/* ==================== 全局共享实例（单电机系统） ==================== */
/* 只显式写 mode：其余字段由静态存储期清零（C 标准保证），
   target/state/device/current/speed/... 的初值由 Control_Init() 填充。
   不再逐个列出 0——那样每加一个字段都会触发 -Wmissing-field-initializers */
MC_Handle_t mc = {
    .mode = MODE_STOP,
};

/* ==================== 模式名称（调试打印） ==================== */
const char *mc_db_mode_name(mode_t mode)
{
    switch (mode)
    {
    case MODE_STOP:                return "STOP";
    case MODE_CURRENT_CALIB:       return "CURRENT_CALIB";
    case MODE_ENCODER_CALIB:       return "ENCODER_CALIB";
    case MODE_FULL_OPEN_LOOP:      return "FULL_OPEN_LOOP";
    case MODE_HALF_OPEN_LOOP:      return "HALF_OPEN_LOOP";
    case MODE_CURRENT_LOOP:        return "CURRENT_LOOP";
    case MODE_SPEED_CURRENT_LOOP:  return "SPEED_CURRENT_LOOP";
    case MODE_POS_SPEED_CURRENT_LOOP: return "POS_SPEED_CURRENT_LOOP";
    default:                       return "UNKNOWN";
    }
}
