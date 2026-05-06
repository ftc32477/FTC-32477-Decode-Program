package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo; // 切换为标准舵机类
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Dual_Servo_Angle_Test", group = "Test")
public class Dual_Servo_Angle_Test extends LinearOpMode {

    // --- 1. 定义对象 ---
    private Servo servo1;
    private Servo servo2;

    // 设定初始位置为 0
    private double currentPosition = 0.0;
    // 计算 20 度对应的位置增量 (假设舵机总行程为 180 度)
    private final double ANGLE_INCREMENT = 20.0 / 180.0;

    // 用于检测按键单次点击的变量（防止按住按键时角度疯狂增加）
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // --- 2. 初始化模块 ---
        // 这里的名称需与 Robot Config 一致
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // 要求 1: 初始化时回到零点[cite: 1]
        servo1.setPosition(0.0);
        servo2.setPosition(0.0);

        telemetry.addLine("状态: 已初始化并归零");
        telemetry.update();

        waitForStart();

        // --- 3. 运行模块 ---
        while (opModeIsActive()) {

            // 要求 2: 按下 X 键，两颗舵机同时正转 20 度
            if (gamepad1.x && !lastX) {
                currentPosition += ANGLE_INCREMENT;
            }

            // 要求 2: 按下 Y 键，两颗舵机同时反转 20 度
            if (gamepad1.y && !lastY) {
                currentPosition -= ANGLE_INCREMENT;
            }

            // 限制范围在 0.0 到 1.0 之间，防止程序溢出
            currentPosition = Range.clip(currentPosition, 0.0, 1.0);

            // 更新舵机位置
            servo1.setPosition(currentPosition);
            servo2.setPosition(currentPosition);

            // 更新按键状态（边缘检测逻辑）
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // 数据监控
            telemetry.addData("当前目标角度(0.0-1.0)", "%.2f", currentPosition);
            telemetry.addData("对应估计角度", "%.1f°", currentPosition * 180);
            telemetry.update();
        }
    }
}