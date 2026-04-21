package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test_Servo_Step", group = "Test")
public class Test_Servo_Step extends LinearOpMode {

    private ServoImplEx servo1;
    // 定义当前目标位置 (0.0 到 1.0)
    private double targetPosition = 0.0;
    // 用于记录上一次 A 键的状态，防止连发
    private boolean lastGamepad1A = false;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");

        // 按照厂家建议设置 PWM 范围以获得最大角度 (约 360 度)
        servo1.setPwmRange(new PwmControl.PwmRange(300, 2700));

        telemetry.addData("状态", "初始化完成");
        telemetry.addLine("按 A 键顺时针旋转 60 度");
        telemetry.update();

        // 初始位置归零
        servo1.setPosition(targetPosition);

        waitForStart();

        while (opModeIsActive()) {
            // 检测 A 键的“上升沿”（即刚按下的那一瞬间）
            if (gamepad1.a && !lastGamepad1A) {

                // 计算步进值：
                // 如果 0.0 到 1.0 对应 360 度，那么 60 度对应的位置增量就是：
                // $$Increment = \frac{60}{360} \approx 0.1667$$
                targetPosition += (60.0 / 360.0);

                // 限制最大值，防止超过 PWM 定义范围
                if (targetPosition > 1.0) {
                    targetPosition = 1.0;
                    telemetry.speak("已到达物理极限");
                }
            }

            if (gamepad1.b) {
                targetPosition = 0.0;
                servo1.setPosition(targetPosition);
            }

            // 更新按键状态
            lastGamepad1A = gamepad1.a;

            // 执行控制
            servo1.setPosition(targetPosition);

            // 显示数据
            telemetry.addData("当前目标值 (0-1)", "%.4f", targetPosition);
            telemetry.addData("估算当前角度", "%.1f°", targetPosition * 360);
            telemetry.update();
        }
    }
}