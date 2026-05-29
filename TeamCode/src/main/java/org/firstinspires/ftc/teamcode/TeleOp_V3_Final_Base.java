package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 战车手动最终版 - 核心控制基类（纯净手动回归版）
 * * 变更说明：
 * 1. 完全移除了 A/B 键的场地绝对角度闭环自瞄逻辑，回归 100% 纯手动摇杆控制。
 * 2. 保留了底盘的 BRAKE 稳态阻尼特性及完整的安全挂起与双模式切换链。
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

    // 保留变量声明，防止 RED/BLUE 子程序由于找不到变量而编译报错
    protected double targetAngleA = 0.0;
    protected double targetAngleB = 0.0;

    @Override
    public void runOpMode() {
        // 1. 初始化底层硬件映射
        robot.init(hardwareMap);

        // 确保底盘处于 BRAKE 状态，在手动摇杆归零时提供最高稳态阻尼
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
            // ==========         2. 获取实时里程计与IMU数据                      ==========
            // ===================================================================
            double rawHubYaw = robot.hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rawOdoYaw = 0.0; double odoX = 0.0; double odoY = 0.0;
            if (robot.ppointOdo != null) {
                rawOdoYaw = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }

            // ===================================================================
            // ==========     3. 底盘麦轮摇杆纯手动控制（已移除自瞄）              ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // 摇杆死区过滤
            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            // 手动推力与旋转平滑控制：平方 Expo 曲线，使中低速操控更细腻
            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);
            turn = Math.signum(turn) * (turn * turn);

            String targetLogStatus = "MANUAL CONTROL";

            // 麦轮动力复合解算
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            // 等比例限幅（防止复合计算后功率绝对值超过 1.0 导致动作失真）
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
        telemetry.addLine("============ 32477 PURE MANUAL INTERFACE ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("Chassis Mode", targetLogStatus);
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
}