package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TeleOp_V3_Final_Base extends LinearOpMode {

    protected RobotHardwareV3 robot = new RobotHardwareV3();
    protected IntakeController intakeManager;
    protected ShooterController shooterManager;

    private int currentGear = 1;
    private final double STICK_DEADZONE = 0.08;

    private int driveMode = 0;
    private boolean lastLBState = false;
    private boolean systemActivated = false;

    // 无头模式控制旗标（默认开启）
    protected boolean isFieldCentric = true;
    private boolean lastOptionsState = false; // 用于捕捉 options (Start) 键的边缘触发

    // 操作手视角的偏航角软件补偿（红蓝方子程序独立配置）
    protected double angleOffset = 0.0;

    protected double targetAngleA = 0.0;
    protected double targetAngleB = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        waitForStart();

        while (opModeIsActive()) {
            if (robot.ppointOdo != null) {
                robot.ppointOdo.update();
            }

            // 1. 获取 Pinpoint 里程计的航向角与坐标
            double rawHeading = 0.0;
            double odoX = 0.0; double odoY = 0.0;
            if (robot.ppointOdo != null) {
                rawHeading = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }
            double odoHeading = normalizeAngle(rawHeading);

            // 动态切换无头/有头模式：利用 gamepad1.options (Xbox 上的 Start 键)
            boolean currentOptionsState = gamepad1.options;
            if (currentOptionsState && !lastOptionsState) {
                isFieldCentric = !isFieldCentric;
            }
            lastOptionsState = currentOptionsState;

            // 原本的开场解锁功能：完美保留，纯粹接管 systemActivated
            if (gamepad1.back) {
                systemActivated = true;
            }

            // 2. 摇杆原始数据采集与死区过滤
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            // 平滑映射
            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            // Field-Centric 无头模式及视线对齐解算
            if (isFieldCentric) {
                // 车体航向角 叠加 红/蓝方操作手视角的偏置补偿 angleOffset
                double driverRelativeHeading = normalizeAngle(odoHeading + angleOffset);

                // 转换为弧度制进行旋转矩阵解算
                double botHeadingRad = Math.toRadians(driverRelativeHeading);

                double rotX = driveX * Math.cos(-botHeadingRad) - driveY * Math.sin(-botHeadingRad);
                double rotY = driveX * Math.sin(-botHeadingRad) + driveY * Math.cos(-botHeadingRad);

                driveX = rotX;
                driveY = rotY;
            }

            String targetLogStatus = "MANUAL TURN";

            // 3. 高精度 P 自动自瞄转向（一键锁头）
            if (gamepad1.a || gamepad1.b) {
                double targetLoc = gamepad1.a ? targetAngleA : targetAngleB;
                double angleError = normalizeAngle(odoHeading - targetLoc);
                double absError = Math.abs(angleError);

                if (absError < 1.0) {
                    turn = 0.0;
                    targetLogStatus = String.format("🔒 LOCK [🎯 TARGET: %.1f°]", targetLoc);
                } else {
                    double kp = 0.015;
                    double MAX_TURN_POWER = 0.8;
                    double pPower = angleError * kp;
                    turn = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, pPower));
                    targetLogStatus = String.format("📉 P-ALIGN [🎯 TARGET: %.1f° | ERR: %.1f°]", targetLoc, angleError);
                }
            } else {
                turn = Math.signum(turn) * (turn * turn);
            }

            // 4. 麦克纳姆轮底盘动力学解算
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            // 功率归一化
            double maxChassisPower = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)), Math.max(Math.abs(lbPower), Math.abs(rbPower)));
            if (maxChassisPower > 1.0) {
                lfPower /= maxChassisPower; rfPower /= maxChassisPower;
                lbPower /= maxChassisPower; rbPower /= maxChassisPower;
            }

            robot.lf.setPower(lfPower); robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower); robot.rb.setPower(rbPower);

            // 5. 安全挂空挡保护
            if (!systemActivated) {
                robot.intake.setPower(0.0); robot.load.setPower(0.0);
                robot.s1.setPower(0.0); robot.s2.setPower(0.0);
                robot.aservo1.setPosition(0.4); robot.aservo2.setPosition(0.4);
                drawTelemetry(false, "NEUTRAL", "WAITING ACTIVATION (PRESS BACK)", targetLogStatus, odoHeading, odoX, odoY);
                continue;
            }

            // 6. 业务控制链分发（吸球/发射逻辑保持不变）
            boolean currentLBState = gamepad1.left_bumper;
            if (currentLBState && !lastLBState) {
                driveMode = (driveMode == 0) ? 1 : 0;
            }
            lastLBState = currentLBState;

            if (gamepad1.dpad_down) currentGear = 1;
            else if (gamepad1.dpad_left) currentGear = 2;
            else if (gamepad1.dpad_right) currentGear = 3;
            else if (gamepad1.dpad_up) currentGear = 4;

            boolean isRTPressed = gamepad1.right_trigger > 0.1;

            if (driveMode == 0) {
                intakeManager.runIntakeMode();
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                intakeManager.stopOrLock(true);
                shooterManager.updateShooter(currentGear, true, isRTPressed);

                if (!shooterManager.isIntercepting() && !isRTPressed) {
                    robot.intake.setPower(0.0);
                    robot.load.setPower(0.0);
                }
            }

            String driveModeStr = (driveMode == 0) ? "📥 INTAKE ACTIVE" : "🚀 SHOOT READY";
            drawTelemetry(true, driveModeStr, "RUNNING", targetLogStatus, odoHeading, odoX, odoY);
        }
    }

    private void drawTelemetry(boolean active, String modeStr, String sysLockStr, String targetLogStatus, double odoHeading, double odoX, double odoY) {
        telemetry.addLine("============ 32477 HIGH PRECISION P-ALIGN Base ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("★ CHASSIS DRIVE MODE", isFieldCentric ? "🌐 FIELD-CENTRIC (无头模式)" : "🤖 ROBOT-CENTRIC (有头模式)");
        telemetry.addData("-> Driver View Offset", "%.1f °", angleOffset);
        telemetry.addData("Chassis Lock Status", targetLogStatus);
        telemetry.addLine("----------------------------------------------------");

        if (active) {
            telemetry.addData("Intake Status", intakeManager.intakeStatus);
            telemetry.addData("S1 RPM", "%.1f", shooterManager.getShooter1RPM());
            telemetry.addData("S2 RPM", "%.1f", shooterManager.getShooter2RPM());
            if (gamepad1.right_trigger > 0.1) {
                telemetry.addData("🔥 Shoot Intercept Hold", "%.2f / 0.50 s", shooterManager.getShootHoldTime());
            }
        }
        telemetry.addLine("----------------------------------------------------");
        telemetry.addLine("[🤖 PPOINT ODO SINGLE SOURCE]");
        telemetry.addData(" -> Raw Pinpoint Heading", "%.2f °", odoHeading);
        telemetry.addData(" -> Compensated Heading", "%.2f °", normalizeAngle(odoHeading + angleOffset));
        telemetry.addData(" -> Pinpoint Local Pos", "X: %.1f cm | Y: %.1f cm", odoX, odoY);
        telemetry.addLine(" -> [Tip] 按手柄中部右侧 Start 键可随时切换 有头/无头 驾驶模式");
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}