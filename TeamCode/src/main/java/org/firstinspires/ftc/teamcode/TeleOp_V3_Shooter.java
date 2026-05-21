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
    private final double BANGBANG_TRIGGER_THRESHOLD = 50.0; // 跌破150RPM触发补速
    private final double BANGBANG_HOLD_DURATION = 1.0;       // 狂暴模式持续1.5秒
    private final double RPM_TOLERANCE = 100.0;               // 进球锁判定精度允许误差

    // 初次加速平滑锁
    private final double FIRST_ACCEL_GAP = -50.0;            // 距离目标转速200RPM以内时，才认为初次加速完成
    private boolean isFirstAcceleration = true;              // 初次加速状态锁

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

        telemetry.addLine("32477: V3战车 [RT联动重置状态锁+球道对转] 已就绪");
        telemetry.addLine(">> 核心安全防护机制:");
        telemetry.addLine("   - 松开RT或处于怠速时，初次加速锁【强行闭合】");
        telemetry.addLine("   - 0->1400 以及 1400->2150 阶段均使用 PIDF 平滑过渡，杜绝突击空转");
        telemetry.addLine("   - 支持 RB(吸球) + LB(清弹) 同时按下对转拦截超载球");
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
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1; // 是否按下RT触发“发射模式”

            if (isTriggerPressed) {
                currentTargetSpeed = targetRPM;
            } else {
                // 【核心修改点】只要松开RT退出发射模式（无论是怠速还是彻底关闭）：
                // 1. 强行将初次加速状态锁重置为 true！
                isFirstAcceleration = true;

                // 2. 档位决策维持原样
                if (targetRPM > 0) {
                    currentTargetSpeed = IDLE_RPM;
                } else {
                    currentTargetSpeed = 0;
                }
            }

            // ==================== 5. 射击电机闭环控制 (包含状态锁保护) ====================
            double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
            double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

            if (currentTargetSpeed > 100) {
                double targetTicksPerSec = (currentTargetSpeed / 60.0) * robot.SHOOTER_TICKS_PER_REV;

                double error1 = currentTargetSpeed - actualRPM1;
                double error2 = currentTargetSpeed - actualRPM2;

                // 只有在进入发射模式（按住RT），且双轮转速皆逼近目标转速以内时，才释放初次加速锁
                if (isTriggerPressed && isFirstAcceleration) {
                    if (actualRPM1 >= (targetRPM - FIRST_ACCEL_GAP) && actualRPM2 >= (targetRPM - FIRST_ACCEL_GAP)) {
                        isFirstAcceleration = false; // 1400->2150平滑拉起冲线完成，解开保护
                    }
                }

                // --- 电机 1 逻辑分支 ---
                // 触发Bang-Bang的严格复合前置条件：必须按住RT 且 已经通过了初次起步阶段 且 跌落缺口超标
                if (isTriggerPressed && !isFirstAcceleration && (error1 >= BANGBANG_TRIGGER_THRESHOLD)) {
                    s1BangTimer.reset();
                }

                if (isTriggerPressed && !isFirstAcceleration && (s1BangTimer.seconds() < BANGBANG_HOLD_DURATION)) {
                    robot.s1.setPower(1.0); // 唯有射击补速时才给 1.0 满电压
                } else {
                    robot.s1.setVelocity(targetTicksPerSec); // 其余时候（包括0-1400, 1400-2150拉升）全部走精密 PIDF
                }

                // --- 电机 2 逻辑分支 ---
                if (isTriggerPressed && !isFirstAcceleration && (error2 >= BANGBANG_TRIGGER_THRESHOLD)) {
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
                isFirstAcceleration = true; // 彻底停机时确保状态锁闭合
            }

            // ==================== 6. 自动化出球挡板判定逻辑 ====================
            boolean speedReady = isTriggerPressed && (targetRPM > 500) &&
                    (Math.abs(actualRPM1 - targetRPM) < RPM_TOLERANCE) &&
                    (Math.abs(actualRPM2 - targetRPM) < RPM_TOLERANCE);

            if (speedReady) {
                robot.aservo1.setPosition(0.0);
                robot.aservo2.setPosition(0.0);
            } else {
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);
            }

            // ==================== 7. 俯仰双轴数学镜像 ====================
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0.0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0.0, 1.0);

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // ==================== 8. 球道重构 (解除硬编码相互死锁，支持组合对转) ====================
            String ballTrackStatus = "IDLE";
            double intakePower = 0.0;
            double loadPower = 0.0;

            if (gamepad1.left_trigger > 0.1) {
                // LT 触发联动装填进球（受进球速度安全锁约束）
                if (speedReady) {
                    intakePower = 0.9;
                    loadPower = 0.9;
                    ballTrackStatus = "LT [FIRE]: Normal Feed";
                } else {
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [LOCKED]: Speed Insufficient";
                }
            } else {
                // 如果没有按主进球 LT，分别解析各个独立按键，使其功率实现物理可叠加：

                // 1. 解析 Intake 控制 (RB 单独吸球)
                if (gamepad1.right_bumper) {
                    intakePower = 0.9;
                    ballTrackStatus = "RB [INTAKE ON]";
                }

                // 2. 解析 Load 控制 (LB 强制倒转吐球/清弹)
                if (gamepad1.left_bumper) {
                    loadPower = -0.9;
                    if (gamepad1.right_bumper) {
                        // 如果同时按下了 RB 和 LB，形成物理对滚状态，拦截多余进球
                        ballTrackStatus = "💥 COUNTER-ROTATING: Intercepting Extra Ball!";
                    } else {
                        ballTrackStatus = "LB [REVERSE LOAD]";
                    }
                }
            }

            // 赋予球道电机实时合成动力（未被按下的电机保持零电无阻尼，球可自由推移）
            robot.intake.setPower(intakePower);
            robot.load.setPower(loadPower);

            // ==================== 9. 全数据遥测监控 ====================
            telemetry.addLine("============ 32477 V3 SYSTEM ============");
            telemetry.addData("Odo Heading", "%.2f °", robot.ppointOdo.getHeading());
            telemetry.addData("Odo X", "%.1f cm", robot.ppointOdo.getPosition().getX(DistanceUnit.CM));
            telemetry.addData("Odo Y", "%.1f cm", robot.ppointOdo.getPosition().getY(DistanceUnit.CM));
            telemetry.addLine("--------------------------------");
            telemetry.addData("Preset Target", "%.0f RPM", targetRPM);
            telemetry.addData("Shooter1 Real", "%.1f RPM", actualRPM1);
            telemetry.addData("Shooter2 Real", "%.1f RPM", actualRPM2);

            // 实时反映两路飞轮的动态保护锁和控制状态
            if (currentTargetSpeed <= 100) {
                telemetry.addData("Control Mode", "STANDBY");
            } else if (isFirstAcceleration) {
                telemetry.addData("Control Mode", "🛡 INITIAL ACCELERATION (PIDF Smooth Lock)");
            } else {
                double s1TimeLeft = Math.max(0, BANGBANG_HOLD_DURATION - s1BangTimer.seconds());
                double s2TimeLeft = Math.max(0, BANGBANG_HOLD_DURATION - s2BangTimer.seconds());
                telemetry.addData("S1 Mode", (s1TimeLeft > 0) ? "⚡ BANG-BANG (剩余 " + String.format("%.2f", s1TimeLeft) + "s)" : "⚙ PIDF锁速");
                telemetry.addData("S2 Mode", (s2TimeLeft > 0) ? "⚡ BANG-BANG (剩余 " + String.format("%.2f", s2TimeLeft) + "s)" : "⚙ PIDF锁速");
            }

            telemetry.addData("Fire Interlock", speedReady ? "🟢 GO" : "🔴 LOCK");
            telemetry.addData("BallTrack State", ballTrackStatus);
            telemetry.update();
        }
    }
}