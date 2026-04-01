package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp_Decode_V2", group = "Main")
public class TeleOp_Decode extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    // 计时器用于控制传输动作的时长
    ElapsedTime transferTimer = new ElapsedTime();
    boolean isTransferring = false;
    boolean lastA = false;

    double liftPosition = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

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

            // --- A. Intake (RT吸/LT吐) ---
            if (gamepad2.right_trigger > 0.1) {
                robot.intake.setPower(0.9);
            } else if (gamepad2.left_trigger > 0.1) {
                robot.intake.setPower(-0.9);
            } else {
                robot.intake.setPower(0.0);
            }

            // --- B. Shooter (X开/B关) ---
            if (gamepad2.x) {
                robot.s1.setPower(1.0);
                robot.s2.setPower(1.0);
            } else if (gamepad2.b) {
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);
            }

            // --- C. 传输系统 (Center舵机 + Loader电机) ---
            // 触发逻辑：按下A键且当前没有正在进行的传输动作
            if (gamepad2.a && !lastA && !isTransferring) {
                isTransferring = true;
                transferTimer.reset(); // 重置计时器
            }
            lastA = gamepad2.a;

            if (isTransferring) {
                // 如果在 0.8 秒内
                if (transferTimer.seconds() < 0.8) {
                    robot.loader.setPower(0.7);    // Loader 电机转动
                    robot.center.setPosition(1.0); // 360舵机全速转动
                } else {
                    // 时间到，停止所有动作
                    robot.loader.setPower(0.0);
                    robot.center.setPosition(0.5); // 0.5 在360舵机中代表停止
                    isTransferring = false;
                }
            }

            // --- D. Lift (D-pad上下微调) ---
            if (gamepad2.dpad_up) liftPosition += 0.005;
            else if (gamepad2.dpad_down) liftPosition -= 0.005;

            liftPosition = Math.max(0.0, Math.min(1.0, liftPosition));
            robot.lift.setPosition(liftPosition);

            // ==========================================
            // 3. 遥测
            // ==========================================
            telemetry.addData("传输状态", isTransferring ? "运行中" : "待机");
            telemetry.addData("传输计时", "%.2f s", transferTimer.seconds());
            telemetry.addData("俯仰角", "%.3f", liftPosition);
            telemetry.update();
        }
    }
}