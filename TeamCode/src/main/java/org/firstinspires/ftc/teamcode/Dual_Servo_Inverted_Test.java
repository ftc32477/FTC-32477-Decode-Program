package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Dual_Servo_Inverted_Test", group = "Test")
public class Dual_Servo_Inverted_Test extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private double currentPosition = 0.0;
    private final double ANGLE_INCREMENT = 20.0 / 180.0;

    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // --- 核心修改：设置 servo2 的方向为反向 ---
        // 这样当代码设置 position 为 0 时，servo2 实际会走到它的物理 1.0 位置
        servo2.setDirection(Servo.Direction.REVERSE);

        // 要求 1: 初始化归零
        // 由于上面设置了反向，此时 servo1 处于物理 0，servo2 处于物理 1[cite: 2]
        currentPosition = 0.0;
        servo1.setPosition(currentPosition);
        servo2.setPosition(currentPosition);

        telemetry.addLine("状态: 已初始化");
        telemetry.addLine("Servo1 处于 0, Servo2 处于物理 1 (逻辑 0)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 按下 X，servo1 增加角度，servo2 减小角度（物理层面）
            if (gamepad1.x && !lastX) {
                currentPosition += ANGLE_INCREMENT;
            }

            // 按下 Y，servo1 减小角度，servo2 增加角度（物理层面）
            if (gamepad1.y && !lastY) {
                currentPosition -= ANGLE_INCREMENT;
            }

            currentPosition = Range.clip(currentPosition, 0.0, 1.0);

            // 同时写入相同的逻辑数值[cite: 1]
            servo1.setPosition(currentPosition);
            servo2.setPosition(currentPosition);

            lastX = gamepad1.x;
            lastY = gamepad1.y;

            telemetry.addData("逻辑位置", "%.2f", currentPosition);
            telemetry.addData("Servo1 物理位置", "%.2f", currentPosition);
            telemetry.addData("Servo2 物理位置", "%.2f", (1.0 - currentPosition)); // 反转显示
            telemetry.update();
        }
    }
}