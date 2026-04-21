package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_Servo_360", group = "Test")
public class Test_Servo_360 extends LinearOpMode {

    private Servo servo1;

    @Override
    public void runOpMode() {
        // 使用标准的 Servo 类来初始化
        // 注意：如果你的硬件配置列表里给它起的名字就是 "Full Range Servo"，
        // 请将这里的 "servo1" 替换成 "Full Range Servo"
        servo1 = hardwareMap.get(Servo.class, "servo1");

        telemetry.addData("状态", "初始化完成，等待启动...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 对于被配置为普通 Servo 的全向舵机：
            // 0.5 = 停止
            // > 0.5 到 1.0 = 正转（数值越靠近 1.0 速度越快）
            // < 0.5 到 0.0 = 反转（数值越靠近 0.0 速度越快）

            // 设定为 1.0，让它在程序启动后保持全速旋转
            servo1.setPosition(1.0);

            telemetry.addData("模式", "已配置为 Full Range Servo (调用 Servo 类)");
            telemetry.addData("当前位置信号 (代表转速/方向)", servo1.getPosition());
            telemetry.update();

            // 保持输出即可
            sleep(50);
        }
    }
}