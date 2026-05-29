package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 战车手动最终版 - 核心控制基类（子程序目标点绝对闭环版）
 * * 物理纠正：
 * 1. 靶心对齐：angleOffset 严格基于子程序写入的目标角度（targetAngleA / B）与当前车头角度计算差值。
 * 2. 目标点零速：当车头进入子程序指定目标点的 ±5° 舒适区时，速度立即化为 0，依靠物理 BRAKE 锁死。
 * 3. 25%限速：自瞄最大旋转功率死死锁在 0.25 以内，温柔吸入。
 */
public class TeleOp_V3_Final_Base extends LinearOpMode {

    protected RobotHardwareV3 robot = new RobotHardwareV3();
    protected IntakeController intakeManager;
    protected ShooterController shooterManager;

    private int currentGear = 1;
    private double iCurrentPosition = 0.5;
    private final double STICK_DEADZONE = 0.08;

    private int driveMode = 0;
    private boolean lastLBState = false;

    // 顶级安全总闸：必须按下 Back 键触发后，机械子系统才会脱离空挡工作
    private boolean systemActivated = false;

    // 🌟 由红蓝子程序直接写入的场地绝对目标角度
    protected double targetAngleA = 0.0;
    protected double targetAngleB = 0.0;

    @Override
    public void runOpMode() {
        // 1. 初始化底层硬件映射
        robot.init(hardwareMap);

        // 确保底盘处于 BRAKE 状态，在自瞄修正量归零时保持最高稳态阻尼
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. 双独立管理类并行实例化
        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        waitForStart();

        while (opModeIsActive()) {
            if (robot.ppointOdo != null) {
                robot.ppointOdo.update();
            }

            // ===================================================================
            // ==========         1. 顶级硬隔离安全监控：Back 激活判定          ==========
            // ===================================================================
            if (gamepad1.back) {
                systemActivated = true;
            }

            // ===================================================================
            // ==========         2. 获取实时里程计绝对航向角                     ==========
            // ===================================================================
            double rawHubYaw = robot.hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rawOdoYaw = 0.0; double odoX = 0.0; double odoY = 0.0;
            if (robot.ppointOdo != null) {
                rawOdoYaw = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }

            // ===================================================================
            // ==========     3. 底盘麦轮摇杆控制与子程序目标点高次自瞄融合         ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            // 手动推力平方 Expo 曲线
            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            String targetLogStatus = "MANUAL TURN";

            if (gamepad1.a || gamepad1.b) {
                // 🌟 【严格直连子程序目标点】
                // 如果按下 A 键，直接读取子程序写入的 targetAngleA；如果按下 B 键，直接读取 targetAngleB
                double targetLoc = gamepad1.a ? targetAngleA : targetAngleB;

                // 🌟 用写入的目标点直接作为算差值的终点！
                double angleOffset = normalizeAngle(targetLoc - rawOdoYaw);
                double absOffset = Math.abs(angleOffset);

                if (absOffset < 5.0) {
                    // 🌟 只要进入子程序写入的目标值正负 5 度以内，无条件彻底断电！
                    // 这就是你指定的真正的“目标值零功率点”
                    turn = 0.0;
                    targetLogStatus = String.format("🔒 [🎯 ARRIVED TARGET: %.1f°]", targetLoc);
                } else {
                    // 🌟 高次渐进软着陆：以子程序写入的目标点 ±5° 为绝对零动力起点
                    double cappedOffset = Math.min(absOffset, 90.0);
                    // 归一化映射到 5° 到 90°
                    double normalizedX = (cappedOffset - 5.0) / (90.0 - 5.0);

                    // 3次曲线：越接近写入的目标值，速度死得越快，在 15° 左右功率只剩 1.5% 的极微弱爬行力
                    double cubicFactor = normalizedX * normalizedX * normalizedX;

                    // 限制自瞄最大辅助速度绝对不超过满速的 25% (0.25)
                    double adaptiveCorrection = cubicFactor * 0.25;

                    // 结合方向赋予底盘
                    turn = Math.signum(angleOffset) * adaptiveCorrection;
                    targetLogStatus = String.format("📉 [ALIGNING TO: %.1f° | POW: %.1f%%]", targetLoc, adaptiveCorrection * 100);
                }
            } else {
                // 松开自瞄键，操作手右摇杆重新获得 100% 自由旋转大功率
                turn = Math.signum(turn) * (turn * turn);
            }

            // 麦轮动力复合解算
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            // 等比例限幅
            double maxChassisPower = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );
            if (maxChassisPower > 1.0) {
                lfPower /= maxChassisPower; rfPower /= maxChassisPower;
                lbPower /= maxChassisPower; rbPower /= maxChassisPower;
            }

            robot.lf.setPower(lfPower); robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower); robot.rb.setPower(rbPower);

            // ===================================================================
            // ==========     ⚠️ 4. 顶级隔离拦截：不按 Back 直接切断控制链     ==========
            // ===================================================================
            if (!systemActivated) {
                robot.intake.setPower(0.0);
                robot.load.setPower(0.0);
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);

                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);

                drawTelemetry(false, "NEUTRAL (空挡挂起)", "WAITING ACTIVATION", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
                continue;
            }

            // ===================================================================
            // ==========       5. 正常业务控制链（按下 Back 解冻后执行）        ==========
            // ===================================================================
            boolean currentLBState = gamepad1.left_bumper;
            if (currentLBState && !lastLBState) {
                driveMode = (driveMode == 0) ? 1 : 0;
            }
            lastLBState = currentLBState;

            if (gamepad1.dpad_down) { currentGear = 1; iCurrentPosition = 0.10; }
            else if (gamepad1.dpad_left) { currentGear = 2; iCurrentPosition = 0.50; }
            else if (gamepad1.dpad_right) { currentGear = 3; iCurrentPosition = 0.50; }
            else if (gamepad1.dpad_up) { currentGear = 4; iCurrentPosition = 1.00; }

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            boolean isRTPressed = gamepad1.right_trigger > 0.1;

            if (driveMode == 0) {
                intakeManager.runIntakeMode();
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                intakeManager.stopOrLock(true);
                shooterManager.updateShooter(currentGear, true, isRTPressed);
            }

            String driveModeStr = (driveMode == 0) ? "📥 INTAKE ACTIVE" : "🚀 SHOOT READY";
            drawTelemetry(true, driveModeStr, "RUNNING (全面解冻)", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
        }
    }

    private void drawTelemetry(boolean active, String modeStr, String sysLockStr, String targetLogStatus,
                               double rawHubYaw, double rawOdoYaw, double odoX, double odoY) {
        telemetry.addLine("============ 32477 ABSOLUTE TARGET INTERFACE ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("Chassis Lock Status", targetLogStatus);
        telemetry.addLine("----------------------------------------------------");

        if (active) {
            telemetry.addData("Intake Status", intakeManager.intakeStatus);
            telemetry.addData("S1 RPM", "%.1f", shooterManager.getShooter1RPM());
            telemetry.addData("S2 RPM", "%.1f", shooterManager.getShooter2RPM());
        } else {
            telemetry.addLine("🚨 警告: 战车机构目前处于安全空挡硬锁状态！");
            telemetry.addLine("🚨 请单次敲击【Back】键解冻上层机械子系统！");
        }
        telemetry.addLine("----------------------------------------------------");
        telemetry.addLine("[🤖 DOUBLE IMU & POSITION VECTOR]");
        telemetry.addData(" -> Hub IMU Yaw", "%.2f °", rawHubYaw);
        telemetry.addData(" -> Pinpoint Heading", "%.2f °", rawOdoYaw);
        telemetry.addData(" -> Local Position", "X: %.1f cm | Y: %.1f cm", odoX, odoY);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}