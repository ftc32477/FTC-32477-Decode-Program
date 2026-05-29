package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

            // 1. 面板与自瞄均采用标准的 -180° 到 180° 约束航向
            double rawHeading = 0.0;
            double odoX = 0.0; double odoY = 0.0;
            if (robot.ppointOdo != null) {
                rawHeading = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }
            double odoHeading = normalizeAngle(rawHeading);

            if (gamepad1.back) {
                systemActivated = true;
            }

            // 2. 高精度 1° 死区纯 P 自瞄
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            String targetLogStatus = "MANUAL TURN";

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

            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            double maxChassisPower = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)), Math.max(Math.abs(lbPower), Math.abs(rbPower)));
            if (maxChassisPower > 1.0) {
                lfPower /= maxChassisPower; rfPower /= maxChassisPower;
                lbPower /= maxChassisPower; rbPower /= maxChassisPower;
            }

            robot.lf.setPower(lfPower); robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower); robot.rb.setPower(rbPower);

            // 3. 安全挂空挡
            if (!systemActivated) {
                robot.intake.setPower(0.0); robot.load.setPower(0.0);
                robot.s1.setPower(0.0); robot.s2.setPower(0.0);
                robot.aservo1.setPosition(0.4); robot.aservo2.setPosition(0.4);
                drawTelemetry(false, "NEUTRAL", "WAITING ACTIVATION", targetLogStatus, odoHeading, odoX, odoY);
                continue;
            }

            // 4. 纯净的业务控制链分发
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
                // 吸取模式：关闭发射内核，但俯仰舵机仍在此行代码中根据 currentGear 实时改变角度
                intakeManager.runIntakeMode();
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                // 发射模式：高能起旋并接管发射逻辑
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
        telemetry.addData(" -> Pinpoint Heading (当前角)", "%.2f °", odoHeading);
        telemetry.addData(" -> Pinpoint Local Pos", "X: %.1f cm | Y: %.1f cm", odoX, odoY);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}