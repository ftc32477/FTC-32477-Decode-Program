package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "TeleOp_Shooter_Final_Fixed", group = "Production")
public class Test_TeleOp_0_1 extends LinearOpMode {

    Test_Hardware robot = new Test_Hardware();

    // 射击速度挡位定义
    private double shootSpeed = 0.5;
    private final double SPEED_LOW = 0.3;
    private final double SPEED_MED = 0.5;
    private final double SPEED_HIGH = 0.75;
    private final double SPEED_MAX = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("正在初始化... 请确保机器人静止以校准角度。");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addLine("就绪！角度已强制归零。");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.odo.update();
            Pose2D pos = robot.odo.getPosition();

            // --- 1. 射击速度挡位切换 (左侧方向键) ---
            if (gamepad1.dpad_down)  shootSpeed = SPEED_LOW;
            if (gamepad1.dpad_left)  shootSpeed = SPEED_MED;
            if (gamepad1.dpad_right) shootSpeed = SPEED_HIGH;
            if (gamepad1.dpad_up)    shootSpeed = SPEED_MAX;

            // --- 2. 吸取与送球控制 (LB/LT) ---
            if (gamepad1.left_trigger > 0.1) {
                // LT：送球逻辑
                robot.intake.setPower(0.8);
                robot.load.setPower(0.8);
            } else if (gamepad1.left_bumper) {
                // LB：仅吸球逻辑
                robot.intake.setPower(0.8);
                robot.load.setPower(0);
            } else {
                robot.intake.setPower(0);
                robot.load.setPower(0);
            }

            // --- 3. 射击控制 (RT) ---
            if (gamepad1.right_trigger > 0.1) {
                robot.s1.setPower(shootSpeed);
                robot.s2.setPower(shootSpeed);
            } else {
                robot.s1.setPower(0);
                robot.s2.setPower(0);
            }

            // --- 4. 底盘控制 (横置底盘映射) ---
            double drive  = -gamepad1.left_stick_x;
            double strafe =  gamepad1.left_stick_y;
            double turn   =  gamepad1.right_stick_x;

            double flP = drive + strafe - turn;
            double frP = drive - strafe - turn;
            double blP = drive - strafe + turn;
            double brP = drive + strafe + turn;

            double max = Math.max(Math.abs(flP), Math.max(Math.abs(frP),
                    Math.max(Math.abs(blP), Math.abs(brP))));
            if (max > 1.0) {
                flP /= max; frP /= max; blP /= max; brP /= max;
            }

            robot.lf.setPower(flP);
            robot.rf.setPower(frP);
            robot.lb.setPower(blP);
            robot.rb.setPower(brP);

            // --- 5. 遥测信息 ---
            telemetry.addLine("--- 机构状态 ---");
            telemetry.addData("当前射击档位", "%.2f", shootSpeed);

            telemetry.addLine("\n--- 位姿对比 (X已取反) ---");
            telemetry.addData("X (cm)", "%.1f", -pos.getX(DistanceUnit.CM));
            telemetry.addData("Y (cm)", "%.1f", pos.getY(DistanceUnit.CM));
            telemetry.addData("内置 IMU °", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint °", "%.1f", pos.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}