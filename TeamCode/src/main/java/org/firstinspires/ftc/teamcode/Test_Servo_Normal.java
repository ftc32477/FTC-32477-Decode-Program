package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_Servo_Normal", group = "Test")
public class Test_Servo_Normal extends LinearOpMode {

    private Servo servo0;

    @Override
    public void runOpMode() {
        // 初始化 0 口的普通舵机，硬件配置中的名字必须为 "servo0"
        servo0 = hardwareMap.get(Servo.class, "servo0");

        telemetry.addData("状态", "初始化完成，等待启动...");
        telemetry.update();

        waitForStart();

        double position = 0.0;
        boolean increasing = true;
        // 每次循环位置变化的步长，调整此数值可以改变扫动速度 (建议范围 0.005 - 0.05)
        double step = 0.01;

        while (opModeIsActive()) {
            // 计算下一个位置
            if (increasing) {
                position += step;
                if (position >= 1.0) {
                    position = 1.0;
                    increasing = false; // 到达最大值，开始反向
                }
            } else {
                position -= step;
                if (position <= 0.0) {
                    position = 0.0;
                    increasing = true;  // 到达最小值，开始正向
                }
            }

            // 执行旋转
            servo0.setPosition(position);

            telemetry.addData("目标位置", position);
            telemetry.update();

            // 短暂休眠以控制循环刷新率，配合 step 控制物理旋转速度
            sleep(20);
        }
    }
}