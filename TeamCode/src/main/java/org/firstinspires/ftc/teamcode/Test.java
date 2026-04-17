package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Test", group = "Main")
public class Test extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    // ===== 新增电机 =====
    DcMotorEx M1, M2;

    // 计时器与状态变量
    ElapsedTime loaderTimer = new ElapsedTime();
    boolean isLoaderRunning = false;
    boolean lastLB = false;

    double liftPosition = 0.5;

    final double LOADER_HOME = 0.333;

    @Override
    public void runOpMode() {

        // 初始化机器人硬件
        robot.init(hardwareMap);

        // ===== 初始化 M1 / M2 =====
        M1 = hardwareMap.get(DcMotorEx.class, "M1");
        M2 = hardwareMap.get(DcMotorEx.class, "M2");

        M1.setDirection(DcMotor.Direction.FORWARD);
        M2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // 初始化 loader 位置
        robot.loaderservo.setPosition(LOADER_HOME);
        sleep(500);

        while (opModeIsActive()) {

            // =============================
            // 1. 底盘控制（无头模式）
            // =============================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            robot.lf.setPower((rotY + rotX + rx) / denominator);
            robot.lb.setPower((rotY - rotX + rx) / denominator);
            robot.rf.setPower((rotY - rotX - rx) / denominator);
            robot.rb.setPower((rotY + rotX - rx) / denominator);

            if (gamepad1.y) robot.imu.resetYaw();

            // =============================
            // 2. 机构控制（手柄2）
            // =============================

            // --- A. Intake ---
            if (gamepad2.right_trigger > 0.1) {
                robot.intake.setPower(0.9);
            } else if (gamepad2.right_bumper) {
                robot.intake.setPower(-0.9);
            } else {
                robot.intake.setPower(0.0);
            }

            // --- B. Shooter ---
            if (gamepad2.left_trigger > 0.1) {
                robot.s1.setPower(1.0);
                robot.s2.setPower(1.0);
            } else {
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);
            }

            // --- C. Center ---
            if (gamepad2.a) {
                robot.center.setPosition(1.0);
            } else {
                robot.center.setPosition(0.5);
            }

            // --- D. Loader ---
            if (gamepad2.left_bumper && !lastLB && !isLoaderRunning) {
                isLoaderRunning = true;
                loaderTimer.reset();
            }
            lastLB = gamepad2.left_bumper;

            if (isLoaderRunning) {
                if (loaderTimer.seconds() < 1.0) {
                    robot.loaderservo.setPosition(LOADER_HOME + 0.1);
                    robot.loader.setPower(0.7);
                } else {
                    robot.loaderservo.setPosition(LOADER_HOME);
                    robot.loader.setPower(0.0);
                    isLoaderRunning = false;
                }
            }

            // --- E. Lift ---
            if (gamepad2.dpad_up) liftPosition += 0.005;
            else if (gamepad2.dpad_down) liftPosition -= 0.005;

            liftPosition = Math.max(0.0, Math.min(1.0, liftPosition));
            robot.lift.setPosition(liftPosition);

            // =============================
            // 3.  M1 / M2 控制
            // =============================
            if (gamepad2.a) {
                // 按 A：只转 M1
                M1.setPower(1.0);
                M2.setPower(0.0);
            } else if (gamepad2.b) {
                // 按 B：M1 + M2 一起转
                M1.setPower(1.0);
                M2.setPower(1.0);
            } else {
                // 停止
                M1.setPower(0.0);
                M2.setPower(0.0);
            }

            // =============================
            // 4. 遥测
            // =============================
            telemetry.addData("Loader Status", isLoaderRunning ? "Running" : "Ready");
            telemetry.addData("M1 Power", M1.getPower());
            telemetry.addData("M2 Power", M2.getPower());
            telemetry.update();
        }
    }
}