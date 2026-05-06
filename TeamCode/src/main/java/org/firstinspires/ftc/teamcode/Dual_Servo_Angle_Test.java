package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Dual_Servo_Angle_Test", group = "Test")
public class Dual_Servo_Angle_Test extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private double currentPosition = 0.4;
    private final double ANGLE_INCREMENT = 20.0 / 180.0;

    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // 保持 servo2 反转：逻辑 0 = 物理 1，逻辑 1 = 物理 0[cite: 3]
        servo2.setDirection(Servo.Direction.REVERSE);

        // 初始化到 0.4
        servo1.setPosition(currentPosition);
        servo2.setPosition(currentPosition);

        telemetry.addData("状态", "已初始化至 0.4");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 新增 RB 操作逻辑 ---
            // 当按住 RB 键时，覆盖 X/Y 的角度控制
            if (gamepad1.right_bumper) {
                // 要求：servo1 转到 0，servo2 转到物理 1
                // 此时 servo2 因为设置了 REVERSE，逻辑 0 即代表物理 1[cite: 3]
                servo1.setPosition(0.0);
                servo2.setPosition(0.0);

                telemetry.addLine("RB 按下中: 强制输出位置 0");
            }
            else {
                // --- 原有的 X/Y 角度增量逻辑 ---
                if (gamepad1.x && !lastX) {
                    currentPosition += ANGLE_INCREMENT;
                }
                if (gamepad1.y && !lastY) {
                    currentPosition -= ANGLE_INCREMENT;
                }

                currentPosition = Range.clip(currentPosition, 0.0, 1.0);

                // 更新正常角度
                servo1.setPosition(currentPosition);
                servo2.setPosition(currentPosition);
            }

            lastX = gamepad1.x;
            lastY = gamepad1.y;

            telemetry.addData("当前逻辑位置变量", "%.2f", currentPosition);
            telemetry.addData("Servo1 物理位置", "%.2f", servo1.getPosition());
            telemetry.addData("Servo2 物理位置", "%.2f", (1.0 - servo2.getPosition()));
            telemetry.update();
        }
    }
}