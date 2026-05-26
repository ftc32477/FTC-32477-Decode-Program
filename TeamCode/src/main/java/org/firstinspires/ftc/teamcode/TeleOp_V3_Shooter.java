package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp_V3_Shooter", group = "Production")
public class TeleOp_V3_Shooter extends LinearOpMode {

    RobotHardwareV3 robot = new RobotHardwareV3();

    // ========== 状态参数定义 ==========
    private double targetRPM = 0;
    private double iCurrentPosition = 0.0;
    private final double IDLE_RPM = 1400.0;

    // --- 控速机制参数 ---
    private final double BANGBANG_TRIGGER_THRESHOLD = 70.0;
    private final double BANGBANG_HOLD_DURATION = 0.3;

    // 非对称进球锁门限
    private final double RPM_TOLERANCE_LOWER = 50.0;
    private final double RPM_TOLERANCE_UPPER = 150.0;

    // 初次加速平滑锁
    private final double FIRST_ACCEL_GAP = -50.0;
    private boolean isFirstAcceleration = true;

    // 防误触滤波参数
    private final double LPF_ALPHA = 0.75;
    private double filteredError1 = 0.0;
    private double filteredError2 = 0.0;

    // 延迟前馈专属计时器与可调阈值
    private ElapsedTime feedforwardDelayTimer = new ElapsedTime();
    private final double FEEDFORWARD_DELAY_SEC = 0.25;
    private boolean isTimerReset = true;

    // 独立控制双飞轮的狂暴模式计时器
    private ElapsedTime s1BangTimer = new ElapsedTime();
    private ElapsedTime s2BangTimer = new ElapsedTime();

    // 防走火阀门单向锁与0.5秒机械到位缓冲计时器
    private boolean hasPassedThreshold = false;
    private ElapsedTime valveTimer = new ElapsedTime();
    private final double VALVE_SETTLE_DELAY_SEC = 0.5;
    private boolean isValveTimerReset = true;

    private final double STICK_DEADZONE = 0.08;

    private boolean lastAState = false;
    private boolean lastBState = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        s1BangTimer.reset();
        s2BangTimer.reset();
        feedforwardDelayTimer.reset();
        valveTimer.reset();

        telemetry.addLine("32477: V3战车 [力学平衡蓄弹+LB释放版] 已就绪");
        telemetry.addLine(">> 精确机械意图对齐说明:");
        telemetry.addLine("   1. [RB平衡蓄弹]: 单按RB时，Intake正转(0.9)吸球，Load反转(-0.9)泄力，球不挤门");
        telemetry.addLine("   2. [LT联动推进]: 射击达标推弹时，Intake与Load自动全正转(0.9)强力喂球");
        telemetry.addLine("   3. [动态回速拦截]: 射击时转速若跌破，球道瞬间物理静止，绝不卡阻");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.ppointOdo.update();

            // ==================== 1. 全向底盘控制 ====================
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;
            double turn   = gamepad1.right_stick_x;

            if (Math.abs(driveY) < STICK_DEADZONE) driveY = 0;
            if (Math.abs(driveX) < STICK_DEADZONE) driveX = 0;
            if (Math.abs(turn)   < STICK_DEADZONE) turn = 0;

            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

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

            robot.lf.setPower(lfPower);
            robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower);
            robot.rb.setPower(rbPower);

            // ==================== 2. 射击基础预设档位 ====================
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
                targetRPM = 1750.0;
                iCurrentPosition = 0.0;
            }

            // ==================== 3. A/B 键转速微调 ====================
            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;

            if (currentAState && !lastAState) {
                targetRPM = Range.clip(targetRPM - 100.0, 0.0, 6000.0);
            }
            if (currentBState && !lastBState) {
                targetRPM = Range.clip(targetRPM + 100.0, 0.0, 6000.0);
            }
            lastAState = currentAState;
            lastBState = currentBState;

            // ==================== 4. 目标转速机制决策与发射状态机 ====================
            double currentTargetSpeed = 0;
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1;

            if (isTriggerPressed) {
                currentTargetSpeed = targetRPM;
            } else {
                isFirstAcceleration = true;
                hasPassedThreshold = false;
                isValveTimerReset = true;
                filteredError1 = 0.0;
                filteredError2 = 0.0;

                if (targetRPM > 0) {
                    currentTargetSpeed = IDLE_RPM;
                } else {
                    currentTargetSpeed = 0;
                }
            }

            // ==================== 5. 带有延迟滤波的前馈判定 ====================
            boolean isLTPressed = gamepad1.left_trigger > 0.1;
            boolean isFeedforwardActive = false;

            if (isTriggerPressed && isLTPressed && hasPassedThreshold) {
                if (isTimerReset) {
                    feedforwardDelayTimer.reset();
                    isTimerReset = false;
                }
                if (feedforwardDelayTimer.seconds() >= FEEDFORWARD_DELAY_SEC) {
                    isFeedforwardActive = true;
                }
            } else {
                isTimerReset = true;
            }

            // ==================== 6. 射击电机闭环控制 ====================
            double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
            double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

            if (currentTargetSpeed > 100) {
                double targetTicksPerSec = (currentTargetSpeed / 60.0) * robot.SHOOTER_TICKS_PER_REV;

                double rawError1 = currentTargetSpeed - actualRPM1;
                double rawError2 = currentTargetSpeed - actualRPM2;

                filteredError1 = (LPF_ALPHA * filteredError1) + ((1.0 - LPF_ALPHA) * rawError1);
                filteredError2 = (LPF_ALPHA * filteredError2) + ((1.0 - LPF_ALPHA) * rawError2);

                if (isTriggerPressed && isFirstAcceleration) {
                    if (actualRPM1 >= (targetRPM - FIRST_ACCEL_GAP) && actualRPM2 >= (targetRPM - FIRST_ACCEL_GAP)) {
                        isFirstAcceleration = false;
                    }
                }

                // --- 电机 1 触发与维持 ---
                if (isTriggerPressed && !isFirstAcceleration && (isFeedforwardActive || filteredError1 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    s1BangTimer.reset();
                }
                if (isTriggerPressed && !isFirstAcceleration && (s1BangTimer.seconds() < BANGBANG_HOLD_DURATION)) {
                    robot.s1.setPower(1.0);
                } else {
                    robot.s1.setVelocity(targetTicksPerSec);
                }

                // --- 电机 2 触发与维持 ---
                if (isTriggerPressed && !isFirstAcceleration && (isFeedforwardActive || filteredError2 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    s2BangTimer.reset();
                }
                if (isTriggerPressed && !isFirstAcceleration && (s2BangTimer.seconds() < BANGBANG_HOLD_DURATION)) {
                    robot.s2.setPower(1.0);
                } else {
                    robot.s2.setVelocity(targetTicksPerSec);
                }

            } else {
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

            // ==================== 7. 防走火阀门单向锁与实时速度就绪状态判定 ====================
            boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);

            boolean isSpeedNowReady = isTriggerPressed && (targetRPM > 500) && s1SpeedReady && s2SpeedReady;

            if (isTriggerPressed && !hasPassedThreshold && isSpeedNowReady) {
                hasPassedThreshold = true;
            }

            boolean isBallPathReadyToRelease = false;

            if (hasPassedThreshold) {
                robot.aservo1.setPosition(0.0);
                robot.aservo2.setPosition(0.0);

                if (isValveTimerReset) {
                    valveTimer.reset();
                    isValveTimerReset = false;
                }

                if (valveTimer.seconds() >= VALVE_SETTLE_DELAY_SEC) {
                    isBallPathReadyToRelease = true;
                }
            } else {
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);
                isValveTimerReset = true;
            }

            // ==================== 8. 俯仰双轴数学镜像 ====================
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0.0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0.0, 1.0);

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // ==================== 9. 球道系统 (精确对齐力学平衡蓄弹构想) ====================
            String ballTrackStatus = "IDLE";
            double intakePower = 0.0;
            double loadPower = 0.0;

            if (isLTPressed) {
                // ==================== 核心分支 A：触发射击流 ====================
                if (!hasPassedThreshold) {
                    // 转速尚未第一次达标
                    intakePower = 0.9;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [WAITING SPEED]: Flywheel Spooling...";
                } else if (!isBallPathReadyToRelease) {
                    // 转速达标了，但舵机大门正在开闸，处于0.5s异步盲区等待
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = String.format("LT [VALVE OPENING]: Settle Buffer %.2fs", valveTimer.seconds());
                } else if (!isSpeedNowReady) {
                    // 🔥【动态速度拦截】：吃球导致飞轮失速，触发球道实时物理停滞
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = "⚠️ LT [RPM DROPPED INTERCEPT]: Speed low, pausing feed!";
                } else {
                    // 🔥【符合意图】：转速达标、舵机门开、且速度在线，intake与load同时“全速正转”，把球轰进飞轮！
                    intakePower = 0.9;
                    loadPower = 0.9;
                    ballTrackStatus = "LT [FIRE]: Speed OK! Load & Intake BOTH FORWARD!";
                }
            } else {
                // ==================== 核心分支 B：日常采球与清理流 ====================
                if (gamepad1.right_bumper) {
                    // 🔥【完美实现意图1】：单按 RB 捡球时，Intake 正转吸球，Load 反转泄力
                    // 实现抓进来的第一颗球卡在球道末端，力学抵消保护阀门舵机，绝不卡死！
                    intakePower = 0.9;
                    loadPower = -0.9; // 物理反转泄力锁
                    ballTrackStatus = "RB [BALANCED BALANCE ACCUMULATION]: Intake FW, Load REV";
                } else if (gamepad1.back) {
                    // 彻底清卡阻/整体清弹专用档位（双向全面反转倒退）
                    intakePower = -0.9;
                    loadPower = -0.9;
                    ballTrackStatus = "💥 BACK BUTTON: Emergency Reverse Ejecting!";
                } else {
                    // 默认状态：Intake 0.2 低功微转锁死外层球权，Load 停止
                    intakePower = 0.4;
                    loadPower = 0.0;
                    ballTrackStatus = "⚡ KEEP BALANCE: Intake 0.2 Prevents Ball Leakage";
                }
            }

            // 最终硬件功率物理平铺
            robot.intake.setPower(intakePower);
            robot.load.setPower(loadPower);

            // ==================== 10. 全数据遥测监控 ====================
            telemetry.addLine("============ 32477 V3 SYSTEM ============");
            telemetry.addData("Preset Target", "%.0f RPM", targetRPM);
            telemetry.addData("Shooter1 Real", "%.1f RPM", actualRPM1);
            telemetry.addData("Shooter2 Real", "%.1f RPM", actualRPM2);
            telemetry.addLine("--------------------------------");
            telemetry.addData("Intake Target Power", "%.2f", intakePower);
            telemetry.addData("Load Target Power", "%.2f", loadPower);
            telemetry.addData("Current Ball Path Mode", ballTrackStatus);
            telemetry.addData("LB Key Status", "🔓 UNBOUND & FREE (完全腾空释放)");
            telemetry.update();
        }
    }
}