## 当前工程架构

- 当前工程的 FOC 实时主链路由 `TIM1` 更新中断驱动，入口位于 `Sdrive/tim_drv.c` 的 `HAL_TIM_PeriodElapsedCallback()`；20kHz 中断内顺序为 `GetAdc() -> AxisDq() -> MotorSpeedGet(按 SPEED_CNT 分频) -> MotorModeSel()`，因此采样、坐标变换、速度估算、控制器调度和 SVPWM 输出处于同一条实时链路。
- `App/task.c` 的 `task_init()` 负责启动底座：启动按键/串口接收中断、`TIM1` 基础中断、ADC1 regular DMA、ADC1 injected 转换、TIM1 CH4 触发点、TIM3 编码器，并执行 `AdcSampleOffset()` 完成电流零偏校准；初始化后进入 `TO_ZERO` 做编码器零位校准，再回到 `STOP`。
- `Sdrive/adc_drv.c` 是采样汇聚层：regular ADC 的 `ADC_Value[3]` 主要用于温度、母线电压等辅助量；相电流由 injected 通道 `ADC1->JDR1/JDR2` 读取，按 `CurU = (OffsetCurU - raw_u) * I_GAIN`、`CurW = (OffsetCurW - raw_w) * I_GAIN` 重建，再由 `CurV = -CurU - CurW` 补足三相，并更新运行低通量 `AdcFilPara`。
- `Sdrive/tim_drv.c` 的 `MotorSpeedGet()` 通过 TIM3 编码器差分计算机械转速 `Motor.speed_M_rpm`，再乘极对数得到电角速度 `Motor.speed_E_rpm`；速度计算按 `SPEED_CNT` 分频执行，当前速度环也复用这一反馈。
- `MotorFoc/foc.c` 是纯数学变换层：三相电流先经 Clarke 变换得到 `Alpha/Beta`，再按选定角度经 Park 变换得到 `Ds/Qs`；电流环输出的 `Ud/Uq` 通过 IPark 变回 `Alpha/Beta` 电压，再由 SVPWM 生成 `CCR1~CCR3`。
- `MotorFoc/motor_ctrl.c` 的 `AxisDq()` 是有感/无感共用前处理入口：先用 `AdcPara.CurU/V/W` 做 Clarke，调用 `MotorAngleGet()` 获取编码器角度，同时更新 Flux/HFI/SMO 等观测链；随后根据控制模式选择 `Park.Theta`，标准有感模式使用 `Motor.E_theta`，无感/混合模式使用对应观测角。
- `MotorFoc/sen.c` 的 `CurCloseloop()` 构成最内层电流环：固定 `Id=0`，以 `Park.Ds/Qs` 为反馈，经 `pi_id/pi_iq` 得到 `Ipark.Ds/Qs`，再调用 `IPARK_Cale()` 形成电压矢量。
- `MotorFoc/sen.c` 的 `SpeedCloseloop()` 是速度中环统一入口：先根据 `UseRamp` 处理速度给定，速度模式使用斜坡，位置模式直接限幅；然后按 `SPEED_CNT` 更新 `pi_spd.Ref = final_spd_ref * Motor.P` 和 `pi_spd.Fbk = Motor.speed_E_rpm`。
- 当前 `SpeedCloseloop()` 已支持一键切换 PI/SMC：`MotorFoc/motor_para.h` 中 `SPEED_CTRL_USE_SMC=0` 时使用原速度 PI，`SPEED_CTRL_USE_SMC=1` 时调用 `SmcSpeed_Update()`；两条路径都输出 `IqRef`，再统一进入 `CurCloseloop(iq_ref, Motor.E_theta)`，因此没有改动电流环、坐标变换和 SVPWM 输出层。
- 新增 `MotorFoc/smc_speed.c/.h` 是可拆卸滑模速度控制模块：入口只接收电角速度参考、反馈和 q 轴电流反馈，不直接访问 TIM/ADC/PWM 寄存器；模块内部完成电角速度 rpm 到机械 rad/s 的转换，计算滑模面 `s`、指数趋近律输出、超螺旋扰动估计和最终 `IqRef`。
- `smc_speed` 模块内部将功能拆为三层：速度控制器调度层 `SmcSpeed_Update()`，趋近律层 `SmcExpReachingUpdate()`，扰动观测器层 `SmcSuperTwistingObserverUpdate()`；当前默认实现为指数趋近律 + 超螺旋扰动观测器，后续替换模糊自适应趋近律或其他观测器时优先替换对应静态函数和枚举分支，保持 `SpeedCloseloop()` 调用不变。
- SMC 参数集中放在 `MotorFoc/motor_para.h`：`SMC_IQ_LIMIT` 是独立 q 轴电流限幅，`SMC_REACH_K/SMC_REACH_EPSILON/SMC_BOUNDARY_LAYER` 控制指数趋近律，`SMC_STO_K1/SMC_STO_K2/SMC_DISTURBANCE_LIMIT` 控制超螺旋扰动观测器；当前默认 `SPEED_CTRL_USE_SMC=1`。
- `MotorFoc/motor_para.c` 的 `MotorInit()` 新增 `SmcSpeed_Init(&SmcSpeed)`，与 PI 初始化同级；`MotorFoc/motor_ctrl.c` 的 `VariableClear()` 新增 `SmcSpeed_Reset(&SmcSpeed)`，保证停机、故障停机或重新启动前清除扰动估计和观测速度，避免旧状态冲击下次启动。
- `MotorFoc/sen.c` 的 `PosCloseloop()` 仍通过 `SpeedCloseloop(Pos.P_Out, 0)` 进入速度环，因此当 `SPEED_CTRL_USE_SMC=1` 时位置环也复用 SMC 速度中环；当宏切回 0 时位置环自动复用原 PI 速度中环。
- `MotorFoc/motor_ctrl.c` 的 `MotorModeSel()` 仍是控制总调度器：`SPEED_LOOP` 调用 `SpeedCloseloop(Para.SpeedRef, 1)`，`POS_LOOP` 调用 `PosCloseloop()`，`CUR_LOOP` 直接调用 `CurCloseloop()`，IF/HFI/SMO/Flux 等无感路径保持原有结构，所有控制分支结束后统一调用 `FocSvpwm()` 输出。
- `MotorFoc/motor_ctrl.c` 的 `FocSvpwm()` 在 `SvpwmCal()` 后调用 `DeadtimeCompApply()`，依据 `AdcFilPara.CurU/V/W` 的方向对 `CCR1~CCR3` 做死区补偿；本次 SMC 接入没有改变死区补偿、电流采样、角度选择和 PWM 输出边界。
- `MotorFoc/pid.c` 保留原 PI 控制器实现和 `PiParaInit()`，PI 路径没有删除，作为 SMC 调试失败时的快速回退路径；`App/hmi_app.c` 中位置/速度模式下对 `pi_spd.Kp/Ki` 的调整逻辑也保留，切回 PI 后仍可沿用。
- 工程构建由根目录 `CMakeLists.txt` 通过 `file(GLOB_RECURSE USER_SOURCES "App/*.c" "MotorFoc/*.c" "Sdrive/*.c" ...)` 收集用户源码；新增 `MotorFoc/smc_speed.c` 后需要重新执行一次 CMake configure 才会进入 Ninja 源文件列表。

## 2026-06-22 超螺旋扰动观测器滑模速度环修改建议

- 推荐插入点已经落地在 `MotorFoc/sen.c` 的 `SpeedCloseloop()` 内部，位于速度参考/反馈更新之后、`IqRef` 输出之前；不插入 `AxisDq()`、`CurCloseloop()` 或 `FocSvpwm()`，以保持坐标变换层、电流环层和 PWM 输出层职责单一。
- 推荐控制链路为：`速度参考/斜坡 -> 速度误差 s -> 趋近律中间件 -> 滑模控制器 -> 超螺旋扰动观测器补偿 -> IqRef 限幅 -> CurCloseloop(iq_ref, Motor.E_theta) -> 电流环/SVPWM`。
- 观测器可拆卸接口已经以 `SMC_OBSERVER_MODE` 和 `SmcSuperTwistingObserverUpdate()` 形式预留；当前默认超螺旋扰动观测器，后续 ESO、DOB、负载转矩观测器等应在 `smc_speed.c` 内新增实现，不改 `SpeedCloseloop()`。
- 趋近律可替换接口已经以 `SMC_REACHING_MODE` 和 `SmcExpReachingUpdate()` 形式预留；当前默认指数趋近律，后续模糊自适应趋近律、自适应幂次趋近律等应在 `smc_speed.c` 内新增实现，不改扰动观测器和电流环。
- PI/SMC 一键切换使用 `MotorFoc/motor_para.h` 的 `SPEED_CTRL_USE_SMC`：`0` 为原 PI，`1` 为 SMC。调试新算法前建议保留 PI 参数不删，便于通过单个宏快速回退。
- 单位边界固定为：`SpeedCloseloop()` 外部继续沿用工程现有电角速度 rpm 框架，`smc_speed` 模块内部统一换算为机械角速度 rad/s，出口统一换回 q 轴电流给定 `IqRef`。
- 安全边界固定为：SMC 使用独立 `SMC_IQ_LIMIT`，停机/故障/重启时走 `VariableClear()` 清除 SMC 状态；首次上板建议先降低 `SMC_IQ_LIMIT`，小负载空载验证后再逐步放开。
- 验证顺序建议：先确认 PI 宏路径编译通过，再启用 SMC 空载小电流限幅测试；随后依次验证速度阶跃、低速扰动、正反转、停机复位和位置环复用。运行期建议通过 VOFA/RTT/串口临时观测 `SmcSpeed.out.s`、`disturbance_hat`、`reaching_out`、`iq_ref`。
