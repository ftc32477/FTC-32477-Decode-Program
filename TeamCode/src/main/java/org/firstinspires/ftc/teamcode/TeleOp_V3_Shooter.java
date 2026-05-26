package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 战车核心 TeleOp 控制程序 - 终极调校版
 * * 【机械/操作核心逻辑说明】：
 * 1. LB 按键完全释放，可用于后续扩展功能。
 * 2. 【RB平衡蓄弹档】：日常捡球单按 RB 时，Intake正转(0.9)吸球，Load反转(-0.9)阻尼泄力。
 * 利用力学平衡让球悬停在挡板前，既防卡死，又能防止暴力挤压把防走火门舵机憋过载。
 * 3. 【LT火力和推弹】：按下 LT 射击时，只有飞轮初次越过达标线且舵机开门稳定后，Intake与Load才会同时正转(0.9)喂球。
 * 4. 【动态失速时滞拦截】：射击中途，若由于吃球导致重飞轮实时转速跌破安全线，球道(Intake/Load)立刻瞬时物理停转(0.0)，
 * 杜绝在失速状态下强行喂次发弹导致弹道变软，待 Bang-Bang 瞬时补足速度后自动恢复喂球。
 * 5. 【实时单帧 Bang-Bang】：针对 +250g 重飞轮优化，取消了计时器持续触发和前馈，误差超过 80 RPM 时当前帧直接满功率灌注，
 * 进入 80 RPM 以内立刻交回底层官方 PIDF，完美压制过速(Overshoot)。
 */
@TeleOp(name = "TeleOp_V3_Shooter", group = "Production")
public class TeleOp_V3_Shooter extends LinearOpMode {

    // 引入 32477 标准硬件映射类
    RobotHardwareV3 robot = new RobotHardwareV3();

    // ===================================================================
    // ==========                 核心参数与状态锁              ==========
    // ===================================================================

    private double targetRPM = 0;              // 赛场动态目标转速 (RPM)
    private double iCurrentPosition = 0.0;     // 俯仰舵机当前的数学镜像绝对目标位置 (0.0 - 1.0)
    private final double IDLE_RPM = 1400.0;    // 怠速预热转速：只要设置了目标转速且松开右触发器，飞轮保持低功耗旋转以获得极速起旋响应

    // --- 大惯性重飞轮单帧 Bang-Bang 核心门限 ---
    private final double BANGBANG_TRIGGER_THRESHOLD = 80.0;  // 补速判定线：转速低于目标 80 RPM 时触发当前帧暴力灌流

    // --- 严格放行/拦截门限 (非对称误差带) ---
    private final double RPM_TOLERANCE_LOWER = 35.0;          // 允许射击的转速下限误差 (目标值 - 35 RPM)
    private final double RPM_TOLERANCE_UPPER = 200.0;         // 允许射击的转速上限误差 (目标值 + 200 RPM，放宽以应对惯性过冲)

    // --- 初次加速安全平滑锁 ---
    private final double FIRST_ACCEL_GAP = -50.0;             // 初次加速判定缓冲线
    private boolean isFirstAcceleration = true;               // 初次起旋隔离锁：防止飞轮从0硬启动时瞬间误触发球道送球

    // --- 低通滤波器 (LPF) 参数 ---
    private final double LPF_ALPHA = 0.75;                     // 滤波系数：数值越大，波形越平滑，抗噪能力越强，但会带来微量时滞
    private double filteredError1 = 0.0;                       // 飞轮 1 经过 LPF 滤波后的实时转速误差
    private double filteredError2 = 0.0;                       // 飞轮 2 经过 LPF 滤波后的实时转速误差

    // --- 防走火阀门单向启闭锁与机械到位缓冲 ---
    private boolean hasPassedThreshold = false;               // 单向达标锁：起旋后只有两路飞轮第一次真正进入安全转速带，该锁才会打开
    private ElapsedTime valveTimer = new ElapsedTime();        // 舵机机械偏转时滞计时器
    private final double VALVE_SETTLE_DELAY_SEC = 0.5;         // 挡板偏转物理到位缓冲时间 (0.5秒)，确保大门完全卸防再推进球
    private boolean isValveTimerReset = true;                  // 舵机计时器重置辅助标志位

    // 手柄摇杆物理死区滤波 (防止摇杆中位漂移导致底盘电机微震憋热)
    private final double STICK_DEADZONE = 0.08;

    // 用于 A/B 按键边缘触发判断的状态点 (防止长按按键导致数值疯狂跳变)
    private boolean lastAState = false;
    private boolean lastBState = false;

    @Override
    public void runOpMode() {

        // 初始化底层硬件映射
        robot.init(hardwareMap);

        // 重置所有生命周期计时器
        valveTimer.reset();

        // 操纵手开赛前控制台看板输出
        telemetry.addLine("32477: V3战车 [单帧实时BangBang+力学蓄弹版] 已就绪");
        telemetry.addLine("==========================================");
        telemetry.addLine(">> 调试同学注意 (Panel Debugging Guide):");
        telemetry.addLine("   1. 飞轮增重后惯性极大，已取消 Bang-Bang 的 Hold 维持和前馈触发。");
        telemetry.addLine("   2. 补速逻辑：实时 Error >= 80 RPM 则当前帧满功率，否则归还 PIDF。");
        telemetry.addLine("   3. RB按键：Intake 0.9 正转, Load -0.9 反转 (力学平衡保护舵机)。");
        telemetry.update();

        // 等待裁判按下 Start 键
        waitForStart();

        // 主循环控制流
        while (opModeIsActive()) {

            // 每帧更新 Pinpoint 里程计数据，维持全场定位坐标系
            robot.ppointOdo.update();

            // ===================================================================
            // ========== 1. 底盘全向移动控制方程 (Mecanum Kinematics) ==========
            // ===================================================================
            double driveY = -gamepad1.left_stick_y; // 纵向：前后移动
            double driveX = gamepad1.left_stick_x;  // 横向：左右横移
            double turn   = gamepad1.right_stick_x; // 旋转：绕几何中心自旋

            // 物理死区裁剪
            if (Math.abs(driveY) < STICK_DEADZONE) driveY = 0;
            if (Math.abs(driveX) < STICK_DEADZONE) driveX = 0;
            if (Math.abs(turn)   < STICK_DEADZONE) turn = 0;

            // 麦克纳姆轮逆运动学解算方程
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            // 幅值等比例归一化缩放：防止因多轴联动叠加导致单轮超速超限（功率溢出(>1.0)破坏行进轨迹）
            double maxPower = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );
            if (maxPower > 1.0) {
                lfPower /= maxPower;
                rfPower /= maxPower;
                lbPower /= maxPower;
                rbPower /= maxPower;
            }

            // 输出物理功率至底盘四大电机
            robot.lf.setPower(lfPower);
            robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower);
            robot.rb.setPower(rbPower);

            // ===================================================================
            // ========== 2. 射击机构基础预设档位决策 (Dpad 快速切档) ============
            // ===================================================================
            if (gamepad1.dpad_up) {
                targetRPM = 2150.0;
                iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                targetRPM = 2150.0;
                iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                targetRPM = 2150.0;
                iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                targetRPM = 1600.0;
                iCurrentPosition = 0.0;
            }

            // ===================================================================
            // ========== 3. A/B 按键边缘单次触发 - 转速精度高低微调 ==============
            // ===================================================================
            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;

            // A键按下瞬间：转速递减 100 RPM
            if (currentAState && !lastAState) {
                targetRPM = Range.clip(targetRPM - 100.0, 0.0, 6000.0);
            }
            // B键按下瞬间：转速递增 100 RPM
            if (currentBState && !lastBState) {
                targetRPM = Range.clip(targetRPM + 100.0, 0.0, 6000.0);
            }
            lastAState = currentAState;
            lastBState = currentBState;

            // ===================================================================
            // ========== 4. 目标转速控制流决策与发射状态机升级 ===================
            // ===================================================================
            double currentTargetSpeed = 0;
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1; // 右触发器作为飞轮开火总闸

            if (isTriggerPressed) {
                // 总闸扣下，全力驱动飞轮奔向目标开火速度
                currentTargetSpeed = targetRPM;
            } else {
                // 总闸松开，重置所有的起旋隔离锁、单向放行锁、时滞计数器以及低通滤波器缓冲器
                isFirstAcceleration = true;
                hasPassedThreshold = false;
                isValveTimerReset = true;
                filteredError1 = 0.0;
                filteredError2 = 0.0;

                if (targetRPM > 0) {
                    // 如果设置了目标转速但未扣动总闸，执行低功耗怠速热机，大幅消减正式开火时的加速等待
                    currentTargetSpeed = IDLE_RPM;
                } else {
                    currentTargetSpeed = 0;
                }
            }

            // ===================================================================
            // ========== 5. 飞轮电机单帧实时闭环驱动内核 (已消除过冲危险) ========
            // ===================================================================
            // 实时捕获两台发射电机的编码器反馈速度，换算为标准 RPM 值
            double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
            double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

            if (currentTargetSpeed > 100) {
                // 换算系统底层闭环锁速所需的每秒脉冲数 (Ticks Per Second)
                double targetTicksPerSec = (currentTargetSpeed / 60.0) * robot.SHOOTER_TICKS_PER_REV;

                // 计算当前帧的原始物理绝对误差
                double rawError1 = currentTargetSpeed - actualRPM1;
                double rawError2 = currentTargetSpeed - actualRPM2;

                // 注入一阶低通滤波算法，平滑掉重飞轮突变带来的高频机械噪声和编码器测量毛刺
                filteredError1 = (LPF_ALPHA * filteredError1) + ((1.0 - LPF_ALPHA) * rawError1);
                filteredError2 = (LPF_ALPHA * filteredError2) + ((1.0 - LPF_ALPHA) * rawError2);

                // 解除初次起旋隔离锁：当且仅当两台电机从零起旋、第一次贴近目标线附近时释放
                if (isTriggerPressed && isFirstAcceleration) {
                    if (actualRPM1 >= (targetRPM - FIRST_ACCEL_GAP) && actualRPM2 >= (targetRPM - FIRST_ACCEL_GAP)) {
                        isFirstAcceleration = false;
                    }
                }

                // --- 电机 1 实时单帧状态机决策 (低于80 RPM进行单帧强灌，否则回归底层PIDF) ---
                if (isTriggerPressed && !isFirstAcceleration && (filteredError1 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    robot.s1.setPower(1.0); // 跌破安全阈值(>=80 RPM)，说明吃球失速，当前帧立刻1.0满功率疯狂补速
                } else {
                    robot.s1.setVelocity(targetTicksPerSec); // 其余高精区间、怠速区间完全交还给REV原生PIDF精确死锁
                }

                // --- 电机 2 实时单帧状态机决策 (低于80 RPM进行单帧强灌，否则回归底层PIDF) ---
                if (isTriggerPressed && !isFirstAcceleration && (filteredError2 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    robot.s2.setPower(1.0);
                } else {
                    robot.s2.setVelocity(targetTicksPerSec);
                }

            } else {
                // 彻底停机状态，清除全部运动数据与标志位，防止死区残留
                robot.s1.setVelocity(0);
                robot.s2.setVelocity(0);
                robot.s1.setPower(0);
                robot.s2.setPower(0);
                isFirstAcceleration = true;
                hasPassedThreshold = false;
                isValveTimerReset = true;
                filteredError1 = 0.0;
                filteredError2 = 0.0;
            }

            // ===================================================================
            // ========== 6. 防走火阀门单向锁与实时速度就绪状态判定 ==============
            // ===================================================================
            // 实时单帧检查当前飞轮绝对转速是否完全吻合安全开火射击误差带 (目标-35 <= 实时 <= 目标+200)
            boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);

            // 当前帧飞轮转速是否百分之百就绪
            boolean isSpeedNowReady = isTriggerPressed && (targetRPM > 500) && s1SpeedReady && s2SpeedReady;

            // 只要按下开火键且转速第一次冲入达标区间，将单向大门标志位置真
            if (isTriggerPressed && !hasPassedThreshold && isSpeedNowReady) {
                hasPassedThreshold = true;
            }

            // 决定球道系统物理运动放行的最底层标志位
            boolean isBallPathReadyToRelease = false;

            if (hasPassedThreshold) {
                // 单向大门已打开，命令挡弹阀门舵机瞬间偏转归零（彻底卸防开闸）
                robot.aservo1.setPosition(0.0);
                robot.aservo2.setPosition(0.0);

                // 启动异步时滞计时，给机械开门留出0.5秒时间
                if (isValveTimerReset) {
                    valveTimer.reset();
                    isValveTimerReset = false;
                }

                // 只有当大门开启时间跨过 0.5 秒到位缓冲后，物理通道才算真正意义上的“畅通无阻”
                if (valveTimer.seconds() >= VALVE_SETTLE_DELAY_SEC) {
                    isBallPathReadyToRelease = true;
                }
            } else {
                // 处于未开火、未达标、或松闸状态：阀门舵机死死闭紧(0.4)，扮演防走火的坚固物理大门
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);
                isValveTimerReset = true;
            }

            // ===================================================================
            // ========== 7. 俯仰双轴数学镜像联动防角力 ===========================
            // ===================================================================
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0.0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0.0, 1.0);

            // 完美执行一侧正向、另一侧 (1.0 - 反向) 的镜像联动，杜绝硬件由于安装方向相反导致的对顶憋死
            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // ===================================================================
            // ========== 8. 球道动力学决策系统 (平衡蓄弹 + 失速刹车) ==============
            // ===================================================================
            String ballTrackStatus = "IDLE";   // 遥测看板状态字描述
            double intakePower = 0.0;          // 本帧最终赋给吸球电机的物理功率
            double loadPower = 0.0;            // 本帧最终赋给运球电机的物理功率

            boolean isLTPressed = gamepad1.left_trigger > 0.1; // 左触发器作为推进开火控制流

            if (isLTPressed) {
                // ------------------ 【分支 A：进入射击推弹流】 ------------------
                if (!hasPassedThreshold) {
                    // 转速还从未达到过标线：允许外层Intake正转卷球，但Load严格保持0.0静止，防止走火
                    intakePower = 0.9;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [WAITING SPEED]: Flywheel Spooling...";
                } else if (!isBallPathReadyToRelease) {
                    // 转速到达过了，但舵机门正在开启中，处于0.5s的物理到位盲区：两路全部静止，等待大门全开
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = String.format("LT [VALVE OPENING]: Settle Buffer %.2fs", valveTimer.seconds());
                } else if (!isSpeedNowReady) {
                    // 💥【动态失速拦截器】：大门开了且过了0.5s，但由于连续打球、吃球，导致重飞轮当前帧跌破下限(>35 RPM)！
                    // 球道实施物理刹车，Intake与Load瞬间归零，把后面的球卡在门外！等大惯性飞轮被单帧 Bang-Bang 迅速补满转速。
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = "⚠️ LT [RPM DROPPED INTERCEPT]: Speed low, pausing feed!";
                } else {
                    // 满足所有的放行黄金条件（初次达标+大门全开+当前帧速度安全在线）：
                    // Intake 与 Load 同时疯狂正转(0.9)，两轮合力瞬间将平衡蓄弹区的球暴力砸进飞轮，开火！
                    intakePower = 0.9;
                    loadPower = 0.9;
                    ballTrackStatus = "LT [FIRE]: Speed OK! Load & Intake BOTH FORWARD!";
                }
            } else {
                // ------------------ 【分支 B：日常采球与清理流】 ------------------
                if (gamepad1.right_bumper) {
                    // 💡【精确对齐机械意图 1】：主驾驶单按 RB 捡球。
                    // Intake 正转(0.9)负责源源不断地从地面卷球进入通道；
                    // Load 通道绝不正转，而是反转(-0.9)进行反向摩擦泄力！
                    // 进来的第一颗球推进到大门前时，会在此处形成合力相互抵消的力学平衡，轻柔悬停，绝不卡阻，绝不伤舵机！
                    intakePower = 0.9;
                    loadPower = -0.9;
                    ballTrackStatus = "RB [BALANCED ACCUMULATION]: Intake FW, Load REV";
                } else if (gamepad1.back) {
                    // 遭遇极端卡沙、卡阻时的紧急全局倒车档位：双路全速反转，向车外全力吐清卡弹
                    intakePower = -0.9;
                    loadPower = -0.9;
                    ballTrackStatus = "💥 BACK BUTTON: Emergency Reverse Ejecting!";
                } else {
                    // 没有任何按键动作时的保底平衡微转机制：
                    // Intake 给予 0.2 的极低前正转功率，锁死外层球权防止其向车外意外漏出，Load保持静止
                    intakePower = 0.2;
                    loadPower = 0.0;
                    ballTrackStatus = "⚡ KEEP BALANCE: Intake 0.2 Balance Lock";
                }
            }

            // 将上层状态机解算完成的物理功率平铺至两路球道硬件电机
            robot.intake.setPower(intakePower);
            robot.load.setPower(loadPower);

            // ===================================================================
            // ========== 9. Panel 调试看板与全方位遥测监控 (Telemetry) ==========
            // ===================================================================
            telemetry.addLine("============ 32477 V4 HEAVY TUNED ==========");
            // 转速与闭环监控区
            telemetry.addData("Preset Target Speed", "%.0f RPM", targetRPM);
            telemetry.addData("Shooter1 Real-time", "%.1f RPM", actualRPM1);
            telemetry.addData("Shooter2 Real-time", "%.1f RPM", actualRPM2);
            telemetry.addLine("--------------------------------------------");

            // 实时单帧误差与状态监控区
            telemetry.addData("Filtered Error 1 (LPF)", "%.1f RPM", filteredError1);
            telemetry.addData("Filtered Error 2 (LPF)", "%.1f RPM", filteredError2);
            telemetry.addData("Bang-Bang Active Now", (filteredError1 >= BANGBANG_TRIGGER_THRESHOLD || filteredError2 >= BANGBANG_TRIGGER_THRESHOLD) && isTriggerPressed && !isFirstAcceleration ? "🔥 YES (FULL PUMP)" : "⚙️ NO (PIDF STEADY)");
            telemetry.addData("Dynamic Gate Lock", isSpeedNowReady ? "✔ ALLOWING FEED (放行中)" : "❌ SPEED BLOCKED (失速物理拦截/未就绪)");

            // 球道及机构状态区
            telemetry.addData("Ball Track Current Mode", ballTrackStatus);
            telemetry.addData("Intake Actual Output", "%.2f", intakePower);
            telemetry.addData("Load Actual Output", "%.2f", loadPower);
            telemetry.addData("LB Button Allocation", "🔓 UNBOUND & FREE (彻底释放，可用作后续扩展)");

            // 每一帧结束时刷新看板数据
            telemetry.update();
        }
    }
}