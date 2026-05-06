package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Merged_Servo_Final_v2", group = "Test")
public class Merged_Servo_Final extends LinearOpMode {

    // --- Angle 系列舵机 (受 RB 影响，不受 X/Y 影响) ---
    private Servo aservo1;
    private Servo aservo2;
    private double aCurrentPosition = 0.4; // 始终保持在 0.4，除非按下 RB[cite: 5]

    // --- Inverted 系列舵机 (受 X/Y 影响) ---
    private Servo iservo1;
    private Servo iservo2;
    private double iCurrentPosition = 0.0; // 初始位置 0.0[cite: 6]

    // 通用变量
    private final double ANGLE_INCREMENT = 20.0 / 180.0; //[cite: 5, 6]
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // --- 初始化 Angle 系列 ---
        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        aservo2.setDirection(Servo.Direction.REVERSE); //[cite: 5]
        aservo1.setPosition(aCurrentPosition);
        aservo2.setPosition(aCurrentPosition);

        // --- 初始化 Inverted 系列 ---
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");
        iservo2.setDirection(Servo.Direction.REVERSE); //[cite: 6]
        iservo1.setPosition(iCurrentPosition);
        iservo2.setPosition(iCurrentPosition);

        telemetry.addLine("状态: 已初始化");
        telemetry.addLine("X/Y 仅控制 Inverted 系列");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 按键逻辑：仅更新 Inverted 系列的目标位置变量 ---
            if (gamepad1.x && !lastX) {
                iCurrentPosition += ANGLE_INCREMENT; //[cite: 6]
            }
            if (gamepad1.y && !lastY) {
                iCurrentPosition -= ANGLE_INCREMENT; //[cite: 6]
            }

            // 限制 Inverted 系列范围
            iCurrentPosition = Range.clip(iCurrentPosition, 0.0, 1.0); //[cite: 6]

            // --- 执行 Angle 系列逻辑 (独立控制) ---
            if (gamepad1.right_bumper) {
                // 按住 RB 时：aservo1 到 0, aservo2 到物理 1 (逻辑 0)[cite: 5]
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                // 松开 RB 时：回到初始的 0.4 位置，不受 X/Y 影响[cite: 5]
                aservo1.setPosition(aCurrentPosition);
                aservo2.setPosition(aCurrentPosition);
            }

            // --- 执行 Inverted 系列逻辑 (受 X/Y 影响) ---
            iservo1.setPosition(iCurrentPosition); //[cite: 6]
            iservo2.setPosition(iCurrentPosition); //[cite: 6]

            // 更新按键状态
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // 监控
            telemetry.addData("Inverted 逻辑位置 (X/Y控制)", "%.2f", iCurrentPosition);
            telemetry.addData("Angle 状态", gamepad1.right_bumper ? "RB 激活 (0.0)" : "默认 (0.4)");
            telemetry.update();
        }
    }
}