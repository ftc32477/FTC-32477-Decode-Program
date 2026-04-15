package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp_Decode_V3", group = "Main")
public class TeleOp_Decode extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    // 计时器与状态变量
    ElapsedTime loaderTimer = new ElapsedTime();
    boolean isLoaderRunning = false;
    boolean lastLB = false;

    double liftPosition = 0.5;

    // 设定 60 度的基准位置 (60/180 = 0.333)
    final double LOADER_HOME = 0.333;

    @Override
    public void runOpMode() {
        // 执行包含10秒归位逻辑的初始化
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - Servo Reset Done");
        telemetry.update();

        waitForStart();

        // ==========================================
        // 正式程序开始：立即旋转到 60 度作为初始零点
        // ==========================================
        robot.loaderservo.setPosition(LOADER_HOME);
        sleep(500); // 短暂等待确保舵机到位

        while (opModeIsActive()) {
            // ==========================================
            // 1. 底盘驱动 (无头模式)
            // ==========================================
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

            // ==========================================
            // 2. 机构控制 (Driver 2)
            // ==========================================

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

            // --- D. Loader 控制 (基于 LOADER_HOME 动作) ---
            if (gamepad2.left_bumper && !lastLB && !isLoaderRunning) {
                isLoaderRunning = true;
                loaderTimer.reset();
            }
            lastLB = gamepad2.left_bumper;

            if (isLoaderRunning) {
                if (loaderTimer.seconds() < 1.0) {
                    // 在 60 度基准上，额外顺时针转动一点进行推料
                    robot.loaderservo.setPosition(LOADER_HOME + 0.1);
                    robot.loader.setPower(0.7);
                } else {
                    // 序列结束，回到 60 度位置
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

            // ==========================================
            // 3. 遥测
            // ==========================================
            telemetry.addData("Loader Status", isLoaderRunning ? "Running" : "Ready");
            telemetry.addData("Servo Position", robot.loaderservo.getPosition());
            telemetry.update();
        }
    }
}