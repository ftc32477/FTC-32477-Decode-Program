package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint 稳定性校验测试", group = "Test")
public class OdoStableTest extends LinearOpMode {
    RobotConfig robot = new RobotConfig();

    @Override
    public void runOpMode() {
        telemetry.addLine("正在进行 Odo 静置校验，请勿触摸机器人...");
        telemetry.update();

        // 这里会执行我们刚才写的循环校验逻辑
        robot.init(hardwareMap);

        telemetry.addLine("校验通过！");
        telemetry.addData("Odo 重启尝试次数", robot.odoResetCount);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.odo.update();
            Pose2D pos = robot.odo.getPosition();

            telemetry.addData("重启次数", robot.odoResetCount);
            telemetry.addLine("--- 实时位姿 ---");
            telemetry.addData("X (mm)", "%.2f", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.2f", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading", "%.2f °", pos.getHeading(AngleUnit.DEGREES));

            // 如果你在操作中发现又开始漂移，可以手动按 A 再次触发校准
            if (gamepad1.a) {
                robot.odo.resetPosAndIMU();
                robot.odoResetCount++;
            }

            telemetry.update();
        }
    }
}