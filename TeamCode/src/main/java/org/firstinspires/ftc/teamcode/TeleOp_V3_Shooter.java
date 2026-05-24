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
    private final double BANGBANG_TRIGGER_THRESHOLD = 70.0;  // 针对正常波动的稳态滤波阈值
    private final double BANGBANG_HOLD_DURATION = 1.0;        // 狂暴模式最大持续时间

    // 非对称进球锁门限
    private final double RPM_TOLERANCE_LOWER = 45.0;
    private final double RPM_TOLERANCE_UPPER = 250.0;

    // 初次加速平滑锁
    private final double FIRST_ACCEL_GAP = -50.0;
    private boolean isFirstAcceleration = true;

    // 防走火阀门初次达标单向锁
    private boolean hasPassedThreshold = false;

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

    private final double STICK_DEADZONE = 0.08;

    private boolean lastAState = false;
    private boolean lastBState = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        s1BangTimer.reset();
        s2BangTimer.reset();
        feedforwardDelayTimer.reset();

        telemetry.addLine("32477: V3战车 [安全截断自适应前馈版] 已就绪");
        telemetry.addLine(">> 安全核心修复:");
        telemetry.addLine("   - [松开RT急停]: 解决Bang-Bang期间松开RT导致飞轮超速飙车漏洞");
        telemetry.addLine("   - [机制联动]: 保持0.25s进球前馈延迟、低通滤波与非对称进球锁");
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
                targetRPM = 1600.0;
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

            // ==================== 6. 射击电机闭环控制 (引入驾驶员意图安全截断) ====================
            double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
            double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

            if (currentTargetSpeed > 100) {
                double targetTicksPerSec = (currentTargetSpeed / 60.0) * robot.SHOOTER_TICKS_PER_REV;

                double rawError1 = currentTargetSpeed - actualRPM1;
                double rawError2 = currentTargetSpeed - actualRPM2;

                // 一阶低通滤波
                filteredError1 = (LPF_ALPHA * filteredError1) + ((1.0 - LPF_ALPHA) * rawError1);
                filteredError2 = (LPF_ALPHA * filteredError2) + ((1.0 - LPF_ALPHA) * rawError2);

                // 初次起步过渡锁
                if (isTriggerPressed && isFirstAcceleration) {
                    if (actualRPM1 >= (targetRPM - FIRST_ACCEL_GAP) && actualRPM2 >= (targetRPM - FIRST_ACCEL_GAP)) {
                        isFirstAcceleration = false;
                    }
                }

                // --- 电机 1 触发与维持决策 ---
                if (isTriggerPressed && !isFirstAcceleration && (isFeedforwardActive || filteredError1 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    s1BangTimer.reset();
                }

                // 【核心修复】：追加 isTriggerPressed 条件。只有在按着RT射击时，Bang-Bang计时维持才有效。
                // 如果松开 RT (isTriggerPressed == false)，无论计时器剩多少秒，直接硬性截断，强制滚进 else 走标准 PIDF 降速回怠速！
                if (isTriggerPressed && !isFirstAcceleration && (s1BangTimer.seconds() < BANGBANG_HOLD_DURATION)) {
                    robot.s1.setPower(1.0);
                } else {
                    robot.s1.setVelocity(targetTicksPerSec);
                }

                // --- 电机 2 触发与维持决策 ---
                if (isTriggerPressed && !isFirstAcceleration && (isFeedforwardActive || filteredError2 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    s2BangTimer.reset();
                }

                // 【核心修复】电机 2 同步追加安全截断锁
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
                filteredError1 = 0.0;
                filteredError2 = 0.0;
            }

            // ==================== 7. 防走火阀门单向锁动作逻辑 ====================
            boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);

            boolean isSpeedNowReady = isTriggerPressed && (targetRPM > 500) && s1SpeedReady && s2SpeedReady;

            if (isTriggerPressed && !hasPassedThreshold && isSpeedNowReady) {
                hasPassedThreshold = true;
            }

            if (hasPassedThreshold) {
                robot.aservo1.setPosition(0.0);
                robot.aservo2.setPosition(0.0);
            } else {
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);
            }

            // ==================== 8. 俯仰双轴数学镜像 ====================
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0.0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0.0, 1.0);

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // ==================== 9. 球道系统 ====================
            String ballTrackStatus = "IDLE";
            double intakePower = 0.0;
            double loadPower = 0.0;

            if (isLTPressed) {
                if (hasPassedThreshold) {
                    intakePower = 0.9;
                    loadPower = 0.9;
                    ballTrackStatus = "LT [FIRE]: Valve Open, Dumping 3 Balls!";
                } else {
                    intakePower = 0.9;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [INTERCEPT]: Waiting Valve Open";
                }
            } else {
                if (gamepad1.right_bumper) {
                    intakePower = 0.9;
                    ballTrackStatus = "RB [INTAKE ON]";
                }

                if (gamepad1.left_bumper) {
                    loadPower = -0.9;
                    if (gamepad1.right_bumper) {
                        ballTrackStatus = "💥 COUNTER-ROTATING: Intercepting Extra Ball!";
                    } else {
                        ballTrackStatus = "LB [REVERSE LOAD]";
                    }
                }
            }

            robot.intake.setPower(intakePower);
            robot.load.setPower(loadPower);

            // ==================== 10. 全数据遥测监控 ====================
            telemetry.addLine("============ 32477 V3 SYSTEM ============");
            telemetry.addData("Preset Target", "%.0f RPM", targetRPM);
            telemetry.addData("Shooter1 Real", "%.1f RPM", actualRPM1);
            telemetry.addData("Shooter2 Real", "%.1f RPM", actualRPM2);
            telemetry.addLine("--------------------------------");

            if (!isLTPressed) {
                telemetry.addData("Feedforward State", "STANDBY (等待LT)");
            } else if (feedforwardDelayTimer.seconds() < FEEDFORWARD_DELAY_SEC) {
                telemetry.addData("Feedforward State", "⏳ DELAY: %.2fs", feedforwardDelayTimer.seconds());
            } else {
                telemetry.addData("Feedforward State", "🔥 ACTIVE");
            }

            if (currentTargetSpeed <= 100) {
                telemetry.addData("Shooter State", "STANDBY");
            } else if (isFirstAcceleration) {
                telemetry.addData("Shooter State", "🛡 INITIAL ACCELERATION");
            } else {
                double s1TimeLeft = Math.max(0, BANGBANG_HOLD_DURATION - s1BangTimer.seconds());
                double s2TimeLeft = Math.max(0, BANGBANG_HOLD_DURATION - s2BangTimer.seconds());
                telemetry.addData("S1 Mode", (s1TimeLeft > 0 && isTriggerPressed) ? "⚡ BANG-BANG 狂暴" : "⚙ PIDF降速/稳态");
                telemetry.addData("S2 Mode", (s2TimeLeft > 0 && isTriggerPressed) ? "⚡ BANG-BANG 狂暴" : "⚙ PIDF降速/稳态");
            }

            telemetry.addData("Valve Dynamic Gate", hasPassedThreshold ? "🔓 OPENED" : "🔒 LOCKED");
            telemetry.addData("BallTrack State", ballTrackStatus);
            telemetry.update();
        }
    }
}