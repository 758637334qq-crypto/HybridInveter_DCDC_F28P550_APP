/*************************************************************************************************
 * App_HeatFilm.c  —  加热膜使能 + 1 kHz 软 PWM
 *   GPIO27 (HEAT_FILM_GPIO_PWM) : 软件 PWM 输出,由中断翻转
 *   GPIO56 (HEAT_FILM_GPIO_EN)  : 加热膜驱动使能,由 Task 控制
 *************************************************************************************************/
#include "Includes.h"                   // 工程总头文件(GPIO driverlib、类型、HeatFilm_t 定义等)

HeatFilm_t g_stHeatFilm;                // 加热膜全局状态结构体(主循环与中断共享)
                                        // 成员: enable / dutyPct / onTicksChg / onTicksDis / cnt

/* =============================================================================================
 * 函数:HeatFilm_CalcOnTicks   (内部函数)
 * 作用:占空比(%) → 一个 PWM 周期内"输出高电平的中断次数"
 * 公式:onTicks = (duty × period + 50) / 100   (+50 实现四舍五入)
 * 参数:duty   占空比 0~100
 *       period PWM 周期节拍数(充电或放电对应的 TICKS 宏)
 * ============================================================================================= */
static Uint16 HeatFilm_CalcOnTicks(Uint16 duty, Uint16 period)
{
    Uint16 onTicks;                     // 计算结果:高电平节拍数

    // 先强转 32 位再相乘,防止 16 位溢出(100×65535 会超 16 位);+50UL 四舍五入;/100 还原百分比
    onTicks = (Uint16)((duty * period + 50UL) / 100UL);

    if (onTicks > period)               // 保护:duty 异常大时不让高电平节拍数超过周期
    {
        onTicks = period;               // 限幅到 100%
    }
    return onTicks;                     // 返回高电平节拍数
}

/* =============================================================================================
 * 函数:HeatFilm_RefreshOnTicks   (内部函数)
 * 作用:按当前 dutyPct,同时刷新"充电模式"和"放电模式"两套高电平节拍数
 *       中断里直接取现成值,不用在中断里做乘除法,省时间
 * ============================================================================================= */
static void HeatFilm_RefreshOnTicks(void)
{
    // 充电模式:周期 HEAT_FILM_TICKS_CHG 个节拍
    g_stHeatFilm.onTicksChg = HeatFilm_CalcOnTicks(g_stHeatFilm.dutyPct, HEAT_FILM_TICKS_CHG);
    // 放电模式:周期 HEAT_FILM_TICKS_DIS 个节拍(两种模式中断频率不同,节拍数不同)
    g_stHeatFilm.onTicksDis = HeatFilm_CalcOnTicks(g_stHeatFilm.dutyPct, HEAT_FILM_TICKS_DIS);
}

/* =============================================================================================
 * 函数:HeatFilm_Init     (上电调用一次；脚配置已在 Board_InitDigitalOut)
 * 作用:仅清软件状态并拉低（调用层，不配复用/方向）
 * ============================================================================================= */
void HeatFilm_Init(void)
{
    HeatFilm_Off();
}

/* =============================================================================================
 * 函数:HeatFilm_Off
 * 作用:彻底关断加热膜:清所有状态,EN 和 PWM 两脚都拉低
 * 顺序:先把 enable=0,中断检测到后会直接 return,不再翻转 PWM 脚,
 *       然后再拉低两个脚,避免被中断覆盖
 * ============================================================================================= */
void HeatFilm_Off(void)
{
    g_stHeatFilm.enable     = 0U;       // ★先关使能标志,让中断停止输出
    g_stHeatFilm.dutyPct    = 0U;       // 占空比清零
    g_stHeatFilm.onTicksChg = 0U;       // 高电平节拍清零
    g_stHeatFilm.onTicksDis = 0U;
    g_stHeatFilm.cnt        = 0U;       // 计数器清零,下次启动从周期开头开始
    GPIO_writePin(HEAT_FILM_GPIO_EN, 0U);   // 驱动使能脚拉低:硬件层面关闭加热膜驱动
    GPIO_writePin(HEAT_FILM_GPIO_PWM, 0U);  // PWM 脚拉低
}

/* =============================================================================================
 * 函数:HeatFilm_SetEnable
 * 作用:单独开/关使能。关闭时同时拉低 PWM 脚、计数器清零
 * 参数:on  非0=开,0=关
 * ============================================================================================= */
void HeatFilm_SetEnable(Uint16 on)
{
    g_stHeatFilm.enable = (on != 0U) ? 1U : 0U;             // 归一化为 0/1,避免传入 2、3 等值
    GPIO_writePin(HEAT_FILM_GPIO_EN, g_stHeatFilm.enable);  // EN 脚跟随 enable 标志
    
    if (g_stHeatFilm.enable == 0U)                          // 如果是关
    {
        GPIO_writePin(HEAT_FILM_GPIO_PWM, 0U);              // PWM 脚拉低(中断已不再管它)
        g_stHeatFilm.cnt = 0U;                              // 计数器清零
    }
}

/* =============================================================================================
 * 函数:HeatFilm_SetDutyPct
 * 作用:设置占空比并刷新两套节拍数
 * 参数:pct0to100  0~100,超过 100 自动限幅
 * ============================================================================================= */
void HeatFilm_SetDutyPct(Uint16 pct0to100)
{
    if (pct0to100 > 100U)               // 限幅
    {
        pct0to100 = 100U;
    }
    g_stHeatFilm.dutyPct = pct0to100;   // 保存占空比
    HeatFilm_RefreshOnTicks();          // 立即重算充/放电两套 onTicks,中断下次就用新值
}

/* =============================================================================================
 * 函数:HeatFilm_Task   (主循环周期调用)
 * 逻辑:上位机热膜命令 ≠ 0 → 使能并保证有占空比;命令 = 0 → 全部关断
 * ============================================================================================= */
void HeatFilm_Task(void)
{
    if (UartProto_GetHotCmd() != 0U)    // 读取串口协议解析出的"加热命令"
    {
        HeatFilm_SetEnable(1U);         // 打开 EN 脚,允许中断输出 PWM
        //最终需要做的就是环路来算占空比，预留
//        if (g_stHeatFilm.dutyPct == 0U) // 占空比还是 0(刚启动)
        {
            HeatFilm_SetDutyPct(100U);  // 默认给 100% 满功率
        }
//        else
//        {
//            HeatFilm_RefreshOnTicks();  // 占空比已有值,仅重算节拍(保证与宏一致)
//        }
    }
    else                                // 无加热命令
    {
        HeatFilm_Off();                 // 关断一切
    }
}

/* =============================================================================================
 * 函数:HeatFilm_Isr   (在控制中断中调用,放 RAM 运行以加快速度、避免 Flash 等待)
 * 作用:软件 PWM 引擎。每次调用 cnt 加 1,按 cnt 与 onTicks 的大小决定 PWM 脚高低
 * 参数:isCharge  非0=充电模式(用 CHG 周期/节拍),0=放电模式(用 DIS 周期/节拍)
 * ============================================================================================= */
#pragma CODE_SECTION(HeatFilm_Isr, ".TI.ramfunc");     // 本函数链接到 RAM 段(必须写在函数定义之前)
void HeatFilm_Isr(Uint16 isCharge)
{
    Uint16 period;                      // 当前模式的 PWM 周期节拍数
    Uint16 onTicks;                     // 当前模式的高电平节拍数

    if (g_stHeatFilm.enable == 0U)      // 未使能:什么都不做,尽快退出(PWM 脚保持关断时的低电平)
    {
        return;
    }

    if (isCharge != 0U)                 // 充电模式:取充电对应参数
    {
        period  = HEAT_FILM_TICKS_CHG;
        onTicks = g_stHeatFilm.onTicksChg;
    }
    else                                // 放电模式:取放电对应参数
    {
        period  = HEAT_FILM_TICKS_DIS;
        onTicks = g_stHeatFilm.onTicksDis;
    }

    if (++g_stHeatFilm.cnt >= period)   // 计数加 1,到达周期末尾就回 0 → 新周期开始
    {                                   // (模式切换时若 cnt 已超过新 period,也在此被拉回 0)
        g_stHeatFilm.cnt = 0U;
    }

    // cnt 在 [0, onTicks-1] 内输出高,其余输出低 → 占空比 = onTicks/period
    GPIO_writePin(HEAT_FILM_GPIO_PWM, (g_stHeatFilm.cnt < onTicks) ? 1U : 0U);
}
