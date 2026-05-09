package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp_Final_Integrated", group = "Test")
public class TeleOp_Final_Integrated extends LinearOpMode {

    // --- 1. 定义硬件对象 ---
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2;
    private Servo iservo1, iservo2;

    // --- 2. 状态变量 -- -
    private double shootSpeed = 1.0;       // 电机功率变量[cite: 7]
    private double aCurrentPosition = 0.4; // Angle系列默认位置[cite: 5]
    private double iCurrentPosition = 0.0; // Inverted系列默认位置[cite: 6]

    private final double ANGLE_INCREMENT = 20.0 / 180.0; // X/Y微调步进[cite: 5]
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // --- 3. 硬件映射与初始化 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");

        // 电机方向设置（引用自 Test_Hardware）[cite: 8]
        s1.setDirection(DcMotor.Direction.REVERSE); //[cite: 8]
        s2.setDirection(DcMotor.Direction.FORWARD); //[cite: 8]

        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        // 舵机方向与初始位置[cite: 5, 6]
        aservo2.setDirection(Servo.Direction.REVERSE); //[cite: 5]
        iservo2.setDirection(Servo.Direction.REVERSE); //[cite: 6]

        aservo1.setPosition(aCurrentPosition); //[cite: 5]
        aservo2.setPosition(aCurrentPosition); //[cite: 5]
        iservo1.setPosition(iCurrentPosition); //[cite: 6]
        iservo2.setPosition(iCurrentPosition); //[cite: 6]

        telemetry.addLine("射击系统就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. 档位切换逻辑 (左侧方向键) ---
            // 1档: 满速/180°, 2档: 满速/90°, 3档: 满速/0°, 4档: 0.75速/0°[cite: 7]
            if (gamepad1.dpad_up) {
                shootSpeed = 1.0; iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                shootSpeed = 1.0; iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                shootSpeed = 1.0; iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                shootSpeed = 0.75; iCurrentPosition = 0.0;
            }

            // --- 5. 电机射击控制 (RT) ---
            if (gamepad1.right_trigger > 0.1) {
                s1.setPower(shootSpeed); //[cite: 7, 8]
                s2.setPower(shootSpeed); //[cite: 7, 8]
            } else {
                s1.setPower(0);
                s2.setPower(0);
            }

            // --- 6. 舵机微调与覆盖逻辑 ---
            // Inverted系列支持 X/Y 手动微调[cite: 6]
            if (gamepad1.x && !lastX) iCurrentPosition += ANGLE_INCREMENT;
            if (gamepad1.y && !lastY) iCurrentPosition -= ANGLE_INCREMENT;
            iCurrentPosition = Range.clip(iCurrentPosition, 0.0, 1.0);

            iservo1.setPosition(iCurrentPosition); //[cite: 6]
            iservo2.setPosition(iCurrentPosition); //[cite: 6]

            // Angle系列受 RB 强制归零控制[cite: 5]
            if (gamepad1.right_bumper) {
                aservo1.setPosition(0.0); // 物理 0[cite: 5]
                aservo2.setPosition(0.0); // 物理 1[cite: 5]
            } else {
                aservo1.setPosition(aCurrentPosition); // 默认 0.4[cite: 5]
                aservo2.setPosition(aCurrentPosition); // 默认逻辑 0.4[cite: 5]
            }

            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // --- 7. 数据反馈 ---
            telemetry.addData("当前档位速度", "%.2f", shootSpeed); //[cite: 7]
            telemetry.addData("iservo 目标位置", "%.2f (约%.0f°)", iCurrentPosition, iCurrentPosition * 180);
            telemetry.addData("aservo 状态", gamepad1.right_bumper ? "RB 强制归零" : "默认 0.4"); //[cite: 5]
            telemetry.update();
        }
    }
}