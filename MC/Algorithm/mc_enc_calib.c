/**
 * @file    mc_enc_calib.c
 * @brief   编码器校准算法实现（方式一 两点锁轴 / 方式二 慢速旋转）
 * @note    原理、参数含义、失步判据的坑见 mc_enc_calib.h；本文件只做状态机与算术，
 *          读角度/写 PWM/落库都由调用方（Control 层）负责
 */
#include "mc_enc_calib.h"
#include "mc_math.h"        /* MC_Math_SinCos：旋转时电角度 → sin/cos（查表法，无 libm） */

/* ==================== 1. 内部常量与工具 ==================== */
#define CAL_2PI     6.283185307179586f  /* 2π：电角度积分步长 / 角度归一 */
#define CAL_PI      3.141592653589793f  /* π：解绕判据 */
#define CAL_HALF_PI (CAL_PI * 0.5f)     /* π/2：两点锁轴第二点的电角度（理论位移 π/(2·pp) 用到） */

/**
 * @brief  角度取模到 [0, 2π)
 * @note   取模用乘倒数，无除法；只用于"已经解绕过的平均值/偏移"这类小量
 */
static float Cal_NormRad(float rad)
{
    rad -= (float)(int)(rad * (1.0f / CAL_2PI)) * CAL_2PI;
    if (rad < 0.0f) {
        rad += CAL_2PI;
    }
    return rad;
}

/**
 * @brief  方式一：机械角解绕 + 波动 min/max 维护 + 拍数递增
 * @param  h     句柄
 * @param  mech  本拍原始机械角 (rad)
 * @note   编码器角归一在 [0,2π)，跨 0/2π 会跳变 2π。直接对原始角取 min/max 或求平均都会错：
 *           · LOCK：轨迹跨边界时算出 ~2π 的假波动 ⇒ "波动 < 阈值"永不成立、白等超时；
 *           · READ：锁轴点落边界附近时，多拍平均会算成 ≈π 的错误角 ⇒ offset 报废。
 *         故按相邻增量累加成连续量：因 unwrapped ≡ mech (mod 2π)，直接对 (mech − unwrapped)
 *         解绕即得真实小增量，不必另存上一拍原始角。
 * @note   ⚠ 已知局限（方式一保留原样、未改）：LOCK 的波动判据用的是**从段首累计**的
 *         first/last，而转子被吸附到锁轴点的过程本身就跨越几十度 ⇒ 该判据实际不成立，
 *         每段都会走满 max_settle（1s）超时。功能不受影响（1s 足够收敛），只是慢。
 *         方式二的"静止判据"改用了**滑动窗**（见下），不再有这个毛病。
 */
static void Cal_Unwrap2P(EncCalib_t *h, float mech)
{
    if (h->tick == 0u)
    {
        h->unwrapped = mech;        /* 每段锁轴的首拍：以当前角为起点 */
        h->first     = mech;        /* min 初值（解绕值） */
        h->last      = mech;        /* max 初值（解绕值） */
    }
    else
    {
        float d = mech - h->unwrapped;

        if (d > CAL_PI)        d -= CAL_2PI;
        else if (d <= -CAL_PI) d += CAL_2PI;

        h->unwrapped += d;

        if (h->unwrapped < h->first) h->first = h->unwrapped;
        if (h->unwrapped > h->last)  h->last  = h->unwrapped;
    }
    h->tick++;
}

/**
 * @brief  方式二：机械角解绕（整段连续；顺带播种静止判据的参照）
 * @param  h      句柄
 * @param  mech   本拍原始机械角 (rad)
 * @param  first  1 = 本段（ALIGN）首拍：播种 unwrapped / rot_mech / 滑动窗参照
 * @note   与方式一同理。注意这里**不**维护 min/max：方式二的"静止"判据用滑动窗
 *         （与 W 拍前的角比较），而不是从段首累计 —— 累计式在"吸附冲击"后永远不成立
 */
static void Cal_UnwrapRot(EncCalib_t *h, float mech, uint8_t first)
{
    if (first != 0u)
    {
        h->unwrapped   = mech;
        h->rot_mech    = mech;      /* 参照点：进入 PRE 时会重新取一次 */
        h->rot_win_ref = mech;      /* 静止判据的滑动窗参照 */
    }
    else
    {
        float d = mech - h->unwrapped;

        if (d > CAL_PI)        d -= CAL_2PI;
        else if (d <= -CAL_PI) d += CAL_2PI;

        h->unwrapped += d;
    }
}

/* ==================== 2. 方式一：两点锁轴 ==================== */
/**
 * @brief  跑一拍两点锁轴校准
 * @note   阶段机：LOCK（锁轴等稳定）→ READ（多拍平均）→ 点 0 则换点 1 重来一遍，
 *         点 1 则算 dir/offset 并置 done。完整推导见 mc_enc_calib.h
 */
static void Cal_Run2Point(EncCalib_t *h, const EncCalib_Cfg_t *cfg,
                          float mech, EncCalib_Out_t *out)
{
    Cal_Unwrap2P(h, mech);

    /* 本拍电压矢量：点 0 = Uα=U（电角度 0°）⇒ (ud,uq)=(U,0)；点 1 = Uβ=U ⇒ (0,U)。
       坐标系固定在 αβ（sin=0、cos=1），两个点都靠 ud/uq 区分 */
    out->sin = 0.0f;
    out->cos = 1.0f;
    if (h->point == 0u) {
        out->ud = cfg->lock_volt;
        out->uq = 0.0f;
    } else {
        out->ud = 0.0f;
        out->uq = cfg->lock_volt;
    }

    switch (h->phase)
    {
    case ENC_CALIB_PHASE_LOCK:      /* ---- 锁轴：固定矢量锁转子 + 等稳定 ---- */
        /* 稳定判据：最小等待已过 && 位置波动 < 阈值；超时强制进读数
           （⚠ 波动用的是累计 min/max，见 Cal_Unwrap2P 的说明：实际上总走超时分支）
           @note 该判据只看"位置是否不再动"，编码器失效/转子被卡住时同样成立
                 （此时角度无声失效）——故点 1 之后还要用位移符号判方向，
                 位移不足则置 suspect 交调用方打印，避免"校准成功但角度是坏的" */
        if (((h->tick >= cfg->min_settle) && ((h->last - h->first) < cfg->stable_err)) ||
            (h->tick >= cfg->max_settle))
        {
            h->phase = ENC_CALIB_PHASE_READ;
            h->n     = 0u;
            h->sum   = 0.0f;
        }
        break;

    case ENC_CALIB_PHASE_READ:      /* ---- 读数：多拍平均机械角 → 点 0 定零点 / 点 1 定方向 ---- */
        h->sum += h->unwrapped;     /* 累加**解绕后**的角（跨 0/2π 边界也不会算错） */
        if (++h->n >= cfg->read_n)
        {
            float mech_avg = Cal_NormRad(h->sum / (float)cfg->read_n);

            if (h->point == 0u)
            {
                /* 点 0（真实电角度 0°）：记下 θmech0，转子再锁到电角度 +90° 一点 */
                h->mech0 = mech_avg;
                h->point = 1u;
                h->phase = ENC_CALIB_PHASE_LOCK;
                h->tick  = 0u;      /* 下一段锁轴重新播种解绕量与 min/max */
                h->n     = 0u;
                h->sum   = 0.0f;
            }
            else
            {
                /* 点 1（真实电角度 +90°）：由两点位移定方向，再由点 0 定偏移 */
                float  dm     = mech_avg - h->mech0;                /* 名义 ±π/(2·pp) */
                float  expect = CAL_HALF_PI / (float)cfg->pole_pairs; /* 理论机械位移 */
                float  admech, off;
                int8_t dir_auto, dir;

                /* 两个机械角各自在 [0, 2π)，差值可能跨过 0/2π 边界 → 解绕到 (−π, π] */
                if (dm > CAL_PI)        dm -= CAL_2PI;
                else if (dm <= -CAL_PI) dm += CAL_2PI;

                admech   = (dm < 0.0f) ? -dm : dm;
                h->dmech = dm;

                dir_auto = (dm >= 0.0f) ? (int8_t)1 : (int8_t)-1;

                if (cfg->dir_force != 0)
                {
                    /* 调试：手动强制方向，跳过自动判据（仍照常给出 dmech 供对照） */
                    h->suspect = 0u;
                    dir = (cfg->dir_force < 0) ? (int8_t)-1 : (int8_t)1;
                }
                else if ((admech < (cfg->dir_min_ratio * expect)) ||
                         (admech > (cfg->dir_max_ratio * expect)))
                {
                    /* 转子没走到第二点（被齿槽/摩擦卡住）或跳到远处平衡点：
                       位移符号不足以判方向 → 按常规方向 +1 兜底并置 suspect，
                       调用方把结果打印出来供人工确认（多为锁轴电压不足/机械卡死） */
                    h->suspect = 1u;
                    dir = 1;
                }
                else
                {
                    h->suspect = 0u;
                    dir = dir_auto;
                }

                /* θmech0 处真实电角度 = 0 → offset = −dir·pp·θmech0（归一到 [0,2π)） */
                off = Cal_NormRad(-(float)dir * (float)cfg->pole_pairs * h->mech0);

                h->out_dir    = dir;
                h->out_offset = off;
                out->done     = 1u;
            }
        }
        break;

    default:
        break;
    }
}

/* ==================== 3. 方式二：慢速旋转 ==================== */
/**
 * @brief  进入 ALIGN 段（软停 + 等静止）
 * @param  h        句柄
 * @param  stage    接下来要测的方向：0 = 正向，1 = 反向
 * @param  f_entry  ALIGN 的入口频率（Hz）：软停车的起点。上电时 0、换向时 ±f、重试时 ∓f
 * @param  cfg      参数
 * @note   一进来就置 rot_tick=0：下一拍解绕会按 `rot_phase==ALIGN && rot_tick==0` 重新播种
 *         （这只是把连续量换个起点，因 pp 为整数，e 只差 2π·整数倍 ⇒ offset 不受影响）
 */
static void Cal_RotEnter(EncCalib_t *h, uint8_t stage, float f_entry, const EncCalib_Cfg_t *cfg)
{
    h->rot_stage   = stage;
    h->rot_f_entry = f_entry;
    h->rot_phase   = ENC_ROT_ALIGN;
    h->rot_tick    = 0u;
    h->rot_win_cnt = cfg->rot_win_ticks;    /* 参照在首拍播种，满一个窗后才判一次 */
}

/**
 * @brief  跑一拍慢速旋转标定
 * @note   每拍顺序：解绕 → 算本拍频率 → 先把指令角推进到"与刚读到的机械角同一时刻"
 *         → 再做测量累加/段切换 → 最后输出本拍矢量。
 *         先积分再测量的顺序很重要：否则会有一整拍（50µs）的斜差混进 offset，
 *         8Hz 下约 0.14° 电角度，而且正反两向**同号**、差分消不掉
 */
static void Cal_RunRot(EncCalib_t *h, const EncCalib_Cfg_t *cfg,
                       float mech, float dt, EncCalib_Out_t *out)
{
    float f = 0.0f;                                             /* 本拍电频率（Hz，带符号） */
    float s = (h->rot_stage == 0u) ? 1.0f : -1.0f;              /* 本方向：+1 正转 / −1 反转 */
    float d, e, step;
    uint8_t first;

    /* ---- 机械角解绕：只在每段 ALIGN 的首拍播种 ---- */
    first = (uint8_t)((h->rot_phase == ENC_ROT_ALIGN) && (h->rot_tick == 0u));
    Cal_UnwrapRot(h, mech, first);

    /* ---- 本拍频率（段间连续、无跳变）---- */
    switch (h->rot_phase)
    {
    case ENC_ROT_ALIGN:
        /* 软停：从入口频率线性降到 0（上电时入口就是 0 ⇒ 一直是静态矢量） */
        f = (h->rot_tick < cfg->rot_ramp_ticks)
              ? (h->rot_f_entry * (1.0f - (float)h->rot_tick / (float)cfg->rot_ramp_ticks))
              : 0.0f;
        break;
    case ENC_ROT_PRE:
        /* 软启动：0 → ±f，之后保持 */
        f = (h->rot_tick < cfg->rot_ramp_ticks)
              ? (s * cfg->rot_freq * (float)h->rot_tick / (float)cfg->rot_ramp_ticks)
              : (s * cfg->rot_freq);
        break;
    case ENC_ROT_MEAS:
        f = s * cfg->rot_freq;
        break;
    default:
        break;
    }

    /* ---- 电角度积分：连续量 rot_ang 供"拟合"，归一量 rot_ang_norm 供查表 ---- */
    step = CAL_2PI * f * dt;
    h->rot_ang      += step;
    h->rot_ang_norm += step;
    if (h->rot_ang_norm >= CAL_2PI) {
        h->rot_ang_norm -= CAL_2PI;
    } else if (h->rot_ang_norm < 0.0f) {
        h->rot_ang_norm += CAL_2PI;
    }
    h->rot_travel += step;
    h->rot_tick++;

    /* ---- 阶段逻辑 ---- */
    switch (h->rot_phase)
    {
    /* ---------------- ALIGN：软停 + 等转子真的静止 ---------------- */
    case ENC_ROT_ALIGN:
        if (h->rot_win_cnt > 0u)
        {
            h->rot_win_cnt--;
        }
        else
        {
            /* 滑动窗满了：用"W 拍前的角"衡量最近是否在动。
               为什么不能用累计 min/max：吸附冲击本身就是几十度的位移，
               累计判据之后再也不会满足 ⇒ 永远走超时（方式一正是这个毛病） */
            float spread = h->unwrapped - h->rot_win_ref;
            if (spread < 0.0f) {
                spread = -spread;
            }

            if ((spread < cfg->stable_err) &&
                (h->rot_tick >= (uint16_t)(cfg->rot_ramp_ticks + cfg->rot_align_min_ticks)))
            {
                /* 静止了：进入本方向的 PRE。dir 的参照点从这一刻算起
                   （这样吸附阶段的位移不会混进"机械净位移"里） */
                h->rot_mech  = h->unwrapped;
                h->rot_phase = ENC_ROT_PRE;
                h->rot_tick  = 0u;
            }
            h->rot_win_ref = h->unwrapped;          /* 刷新参照：下一个满窗再判 */
            h->rot_win_cnt = cfg->rot_win_ticks;
        }

        /* 超时兜底：软停后等太久 ⇒ 记一笔 align 失败，但仍继续（别把上电卡死） */
        if ((h->rot_phase == ENC_ROT_ALIGN) &&
            (h->rot_tick >= (uint16_t)(cfg->rot_ramp_ticks + cfg->rot_align_max_ticks)))
        {
            h->rot_fail |= ENC_FAIL_ALIGN;
            h->rot_mech  = h->unwrapped;
            h->rot_phase = ENC_ROT_PRE;
            h->rot_tick  = 0u;
        }
        break;

    /* ---------------- PRE：升速 + 稳定（只跟踪，不测量） ---------------- */
    case ENC_ROT_PRE:
        if (h->rot_tick >= (uint16_t)(cfg->rot_ramp_ticks + cfg->rot_settle_ticks))
        {
            if (h->rot_stage == 0u)
            {
                /* 正向段结束：由机械净位移的符号定方向
                   （与方式一用"两点位移符号"同理，这里位移长达好几圈，可靠得多） */
                float  dmech    = h->unwrapped - h->rot_mech;
                int8_t dir_auto = (dmech >= 0.0f) ? (int8_t)1 : (int8_t)-1;

                h->rot_dir = (cfg->dir_force != 0)
                               ? ((cfg->dir_force < 0) ? (int8_t)-1 : (int8_t)1)
                               : dir_auto;
            }

            /* 换段：测量段的参照点与累加器重新开始 */
            h->rot_mech   = h->unwrapped;
            h->rot_travel = 0.0f;
            h->rot_sum_e  = 0.0f;
            h->rot_e_min  =  1.0e9f;    /* 残差峰峰值用：先放极端值 */
            h->rot_e_max  = -1.0e9f;
            h->rot_n      = 0u;
            h->rot_phase  = ENC_ROT_MEAS;
            h->rot_tick   = 0u;
        }
        break;

    /* ---------------- MEAS：测量 e = offset ± δe，走够 REVS 圈 ---------------- */
    case ENC_ROT_MEAS:
        e = h->rot_ang - (float)h->rot_dir * (float)cfg->pole_pairs * h->unwrapped;
        h->rot_sum_e += e;
        if (h->rot_stage == 0u) {       /* 残差峰峰值只统计正向段（报告用） */
            if (e < h->rot_e_min) h->rot_e_min = e;
            if (e > h->rot_e_max) h->rot_e_max = e;
        }
        h->rot_n++;

        if (((h->rot_stage == 0u) && (h->rot_travel >=  cfg->rot_travel_target)) ||
            ((h->rot_stage == 1u) && (h->rot_travel <= -cfg->rot_travel_target)) ||
            (h->rot_tick >= cfg->rot_max_meas_ticks))
        {
            float dmech  = h->unwrapped - h->rot_mech;
            float dm_abs = (dmech < 0.0f) ? -dmech : dmech;
            float tr_abs = (h->rot_travel < 0.0f) ? -h->rot_travel : h->rot_travel;
            float ratio  = (tr_abs > 1.0e-6f)
                             ? (dm_abs * (float)cfg->pole_pairs / tr_abs)
                             : 0.0f;    /* 跟随性：同步时应 ≈1.0；丢一步就是 0.857 */
            float off;

            if (h->rot_n == 0u) {       /* 防 0 除（理论到不了） */
                h->rot_n = 1u;
            }
            off = h->rot_sum_e / (float)h->rot_n;

            if (h->rot_tick >= cfg->rot_max_meas_ticks) {
                h->rot_fail |= ENC_FAIL_MEAS_TO;        /* 没走够圈数就超时 */
            }

            if (h->rot_stage == 0u)
            {
                /* 正向段收尾：先回 ALIGN 把 +f 软停到 0 并等静止，再反向测一遍
                   （直接从 +f 穿到 −f 会有一次换向冲击，也是丢步的来源之一）
                   ⚠ e 的**平均值**必须先归一到 [0,2π) 再存：每段 ALIGN 都会用当前机械角
                     重新播种解绕量，这会让 e 整体平移 dir·pp·2π·m（是 2π 的整数倍 ⇒
                     e mod 2π 不变，但**绝对值能差好几圈**：正段转 1 机械圈 ⇒ m≈1 ⇒ 差 7 圈）。
                     不归一的话 off_fwd − off_rev 会差好几整圈，dd/2 就不是 2π 的整数倍，
                     offset 直接偏 180°（实测就是这么翻车的） */
                h->rot_off_fwd   = Cal_NormRad(off);
                h->rot_resid     = h->rot_e_max - h->rot_e_min;     /* 残差峰峰值：本段采完就带走 */
                h->rot_ratio_fwd = ratio;
                Cal_RotEnter(h, 1u, s * cfg->rot_freq, cfg);
            }
            else
            {
                float dd, load;

                h->rot_off_rev   = Cal_NormRad(off);                /* 同上：先归一 */
                h->rot_ratio_rev = ratio;
                h->rot_ratio     = (h->rot_ratio_fwd < ratio) ? h->rot_ratio_fwd : ratio;   /* 取较差 */

                if ((h->rot_ratio < cfg->rot_ratio_min) || (h->rot_ratio > cfg->rot_ratio_max)) {
                    h->rot_fail |= ENC_FAIL_SLIP;       /* 丢步 ⇒ offset 偏差 ≈ 滑移量的一半 */
                }

                /* 两方向**圆平均**：off = off_rev + wrap(off_fwd − off_rev)/2
                   差值的一半就是跟随负载角 δe（正向 e=offset+δe、反向 e=offset−δe）。
                   两者都已在 [0,2π) ⇒ |dd| < 2π，一次解绕就够 */
                dd = h->rot_off_fwd - h->rot_off_rev;
                if (dd > CAL_PI)        dd -= CAL_2PI;
                else if (dd <= -CAL_PI) dd += CAL_2PI;
                load = 0.5f * dd;
                h->rot_load = load;

                if ((load > cfg->rot_max_load) || (load < -cfg->rot_max_load)) {
                    h->rot_fail |= ENC_FAIL_LOAD;
                }

                if ((h->rot_fail != 0u) && (h->rot_attempt < cfg->rot_retry_n))
                {
                    /* 判据没过 → 整段重来。丢步/换向冲击都是**暂态**，重来一次通常就过；
                       重来前先软停到 0 再对齐，避免"正在转就突然给静态矢量"的扭矩冲击。
                       清掉 rot_fail：报告只反映**最后一次**尝试（重试次数看 rot_attempt）；
                       但把**第一次**的原因留一份进 rot_fail_first —— 否则重试成功后
                       就再也查不到"当初为什么重试" */
                    if (h->rot_attempt == 0u) {
                        h->rot_fail_first = h->rot_fail;
                    }
                    h->rot_attempt++;
                    h->rot_fail = 0u;
                    Cal_RotEnter(h, 0u, s * cfg->rot_freq, cfg);    /* s = −1 ⇒ 从 −f 软停 */
                }
                else
                {
                    h->suspect    = (uint8_t)(h->rot_fail != 0u);
                    h->out_dir    = h->rot_dir;
                    h->out_offset = Cal_NormRad(h->rot_off_rev + 0.5f * dd);
                    out->done     = 1u;
                }
            }
        }
        break;

    default:
        break;
    }

    /* ---- 输出：电压矢量必须放在**指令 d 轴**上（ud = rot_volt、uq = 0）----
       ⚠ 这里是本模块最容易搞错、也最难查的一处约定（第一版就错在这里，症状是
         "CAL 行一切正常但电机不转、一直吃电流"）：
           · MC_SVPWM_Calc(ud, uq, s, c) 里 (s,c) 是**转子 d 轴**的电角度、ud/uq 是 dq 电压
             （半开环用编码器角 + uq=V 能转，即验证了这一点）；
           · 而转子 d 轴会**追着定子电流矢量（≈电压矢量）跑**（零转矩的稳定平衡点
             就是电流落在 d 轴上）。
         所以：
           用 (ud=V, uq=0) ⇒ αβ 矢量就在指令角 θ_cmd 上 ⇒ 转子 d 轴 → θ_cmd ✓
                              ⇒ e = θ_cmd − dir·pp·θm = offset_真 + δe ✓
           用 (ud=0, uq=V) ⇒ αβ 矢量在 θ_cmd + 90°   ⇒ 转子 d 轴 → θ_cmd + 90° ✗
                              ⇒ 算出的 offset **整整小 90°** ⇒ 闭环时电流全落在真 d 轴上
                                ⇒ 电机不转但有电流（CAL 行却看起来完全正常，因为
                                  δe/ratio 都不受这 90° 影响）
         （全开环 VVVF 用 (0,V) 也照样能转 —— 它不需要给转子 d 轴贴角度标签，
           所以这个坑只在"要标定 θm → θe 映射"时才暴露） */
    MC_Math_SinCos(h->rot_ang_norm, &out->sin, &out->cos);
    out->ud = cfg->rot_volt;
    out->uq = 0.0f;
}

/* ==================== 4. 对外接口 ==================== */
void EncCalib_Reset(EncCalib_t *h)
{
    /* ---- 方式一 两点锁轴 ---- */
    h->phase   = ENC_CALIB_PHASE_LOCK;
    h->tick    = 0u;
    h->n       = 0u;
    h->first   = 0.0f;
    h->last    = 0.0f;
    h->sum     = 0.0f;
    h->mech0   = 0.0f;
    h->dmech   = 0.0f;
    h->point   = 0u;

    /* ---- 共用 ---- */
    h->unwrapped  = 0.0f;   /* 解绕量：各段 ALIGN/锁轴的首拍会按当前角重新播种 */
    h->suspect    = 0u;
    h->done       = 0u;
    h->out_dir    = 1;
    h->out_offset = 0.0f;

    /* ---- 方式二 慢速旋转 ---- */
    h->rot_phase     = ENC_ROT_ALIGN;
    h->rot_stage     = 0u;
    h->rot_attempt   = 0u;
    h->rot_fail      = 0u;
    h->rot_fail_first = 0u;
    h->rot_dir       = 1;
    h->rot_f_entry   = 0.0f;    /* 上电入口频率 = 0：第一次 ALIGN 就是纯静态吸附 */
    h->rot_tick      = 0u;
    h->rot_n         = 0u;
    h->rot_ang       = 0.0f;
    h->rot_ang_norm  = 0.0f;
    h->rot_travel    = 0.0f;
    h->rot_sum_e     = 0.0f;
    h->rot_e_min     = 0.0f;
    h->rot_e_max     = 0.0f;
    h->rot_resid     = 0.0f;
    h->rot_win_ref   = 0.0f;
    h->rot_win_cnt   = 0u;
    h->rot_mech      = 0.0f;
    h->rot_off_fwd   = 0.0f;
    h->rot_off_rev   = 0.0f;
    h->rot_ratio_fwd = 0.0f;
    h->rot_ratio_rev = 0.0f;
    h->rot_ratio     = 0.0f;
    h->rot_load      = 0.0f;
}

uint8_t EncCalib_Run(EncCalib_t *h, const EncCalib_Cfg_t *cfg,
                     float mech, float dt, EncCalib_Out_t *out)
{
    /* 默认输出：本拍不加电压（只有两个子状态机会按方式改写它） */
    out->ud   = 0.0f;
    out->uq   = 0.0f;
    out->sin  = 0.0f;
    out->cos  = 1.0f;
    out->done = 0u;

    /* 已完成：幂等地把结果报一遍（调用方可能还没走完"关断+写回"那几拍） */
    if (h->done != 0u) {
        out->done = 1u;
        return 1u;
    }

    if (cfg->method == ENC_CALIB_METHOD_ROT) {
        Cal_RunRot(h, cfg, mech, dt, out);
    } else {
        Cal_Run2Point(h, cfg, mech, out);
    }

    if (out->done != 0u) {
        h->done = 1u;
        return 1u;
    }
    return 0u;
}

const char *EncCalib_FailStr(const EncCalib_t *h)
{
    if (h->suspect == 0u) {
        return "ok";
    }
    if ((h->rot_fail & ENC_FAIL_DRV) != 0u) {
        return "SUSPECT(driver-fault)";
    }
    if ((h->rot_fail & ENC_FAIL_ENC) != 0u) {
        return "SUSPECT(encoder-stale)";
    }
    if ((h->rot_fail & ENC_FAIL_ALIGN) != 0u) {
        return "SUSPECT(align-not-settled)";
    }
    if ((h->rot_fail & ENC_FAIL_MEAS_TO) != 0u) {
        return "SUSPECT(no-travel/timeout)";
    }
    if ((h->rot_fail & ENC_FAIL_SLIP) != 0u) {
        return "SUSPECT(slip-lost-steps)";
    }
    if ((h->rot_fail & ENC_FAIL_LOAD) != 0u) {
        return "SUSPECT(load-angle)";
    }
    return "SUSPECT(2point-rotor-not-moved)";
}

const char *EncCalib_FailCode(uint8_t fail)
{
    if ((fail & ENC_FAIL_DRV) != 0u) {
        return "drv";
    }
    if ((fail & ENC_FAIL_ENC) != 0u) {
        return "enc";
    }
    if ((fail & ENC_FAIL_ALIGN) != 0u) {
        return "align";
    }
    if ((fail & ENC_FAIL_MEAS_TO) != 0u) {
        return "no-run";
    }
    if ((fail & ENC_FAIL_SLIP) != 0u) {
        return "slip";
    }
    if ((fail & ENC_FAIL_LOAD) != 0u) {
        return "load";
    }
    return "-";
}

void EncCalib_Abort(EncCalib_t *h, uint8_t fail)
{
    h->rot_fail |= fail;
    h->suspect   = 1u;
    h->done      = 1u;      /* 幂等：之后 Run 直接返回"完成"，不再推进状态 */
    h->out_dir   = 1;       /* 结果无效（调用方不得写回；置成中性值以免被误用） */
    h->out_offset = 0.0f;
}
