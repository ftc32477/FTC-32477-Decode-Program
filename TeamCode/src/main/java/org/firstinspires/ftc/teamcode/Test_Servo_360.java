package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test_Servo_360", group = "Test")
public class Test_Servo_360 extends LinearOpMode {

    private ServoImplEx servo1;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");

        // 设置脉冲范围，确保 1.0 对应 3000us
        servo1.setPwmRange(new PwmControl.PwmRange(500, 3000));

        telemetry.addData("准备就绪", "按下开始键后，将顺时针旋转3秒后停止");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // --- 1. 开始旋转 ---
            // 发送 3000us 信号，进入第二种模式：顺时针连续旋转
            servo1.setPosition(1.0);

            telemetry.addData("状态", "正在连续旋转...");
            telemetry.update();

            // 持续 3000 毫秒（3秒）
            sleep(3000);

            // --- 2. 停止旋转 ---
            // 方式 A：直接禁用 PWM 信号（最保险，舵机将完全失去动力，不会发出嗡嗡声）
            servo1.setPwmDisable();

            // 方式 B：如果你希望它停在某个固定角度而不是完全无力，可以使用下面的代码替换方式 A
            // servo1.setPosition(0.5); // 切换回第一种模式，停在中间角度

            telemetry.addData("状态", "已停止");
            telemetry.update();
        }

        // 保持程序运行直到手动按下停止，防止代码立刻退出
        while (opModeIsActive()) {
            idle();
        }
    }
}