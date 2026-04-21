/*编码器读数程序*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint 1位小数测试", group = "Test")
public class Test_OdoStable extends LinearOpMode {
    RobotConfig robot = new RobotConfig();

    @Override
    public void runOpMode() {
        telemetry.addLine("正在初始化 Pinpoint (精度: 0.1cm)...");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addLine("校验成功！传感器已稳定。");
        telemetry.addData("初始化重启尝试", robot.odoResetCount);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.odo.update();
            Pose2D pos = robot.odo.getPosition();

            // 提取 cm 数值
            double x_cm = pos.getX(DistanceUnit.CM);
            double y_cm = pos.getY(DistanceUnit.CM);
            double heading_deg = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addData("重启次数", robot.odoResetCount);
            telemetry.addLine("--- 实时位姿 (cm) ---");
            // 使用 %.1f 实现 cm 的一位小数显示
            telemetry.addData("X (前进)", "%.1f cm", x_cm);
            telemetry.addData("Y (侧移)", "%.1f cm", y_cm);
            telemetry.addData("Heading", "%.1f °", heading_deg);

            // 实时状态监控
            telemetry.addLine("\n--- 诊断信息 ---");
            telemetry.addData("状态", robot.odo.getDeviceStatus());
            // 速度也用 cm/s 显示，保留一位小数
            telemetry.addData("实时速度", "%.1f cm/s", robot.odo.getVelocity().getX(DistanceUnit.CM));

            if (gamepad1.a) {
                robot.odo.resetPosAndIMU();
            }

            telemetry.update();
        }
    }
}