package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter_Auto_Fire_System", group = "Production")
public class Shooter_Auto_Fire extends LinearOpMode {

    // ========== 1. 硬件对象 ==========
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2;
    private Servo iservo1, iservo2;

    // ========== 2. 常数定义 ==========
    private final double TICKS_PER_REV = 537.7;
    private final double P = 15.0, I = 5.0, D = 1.0, F = 12.5;
    private final double FULL_SPEED_RPM = 100.0;

    // 【新增】转速判定容差：实测转速与目标转速差距小于此值时，视为达标[cite: 2]
    private final double RPM_TOLERANCE = 10.0;

    // ========== 3. 状态变量 ==========
    private double targetRPM = 0;
    private double iCurrentPosition = 0.0;

    @Override
    public void runOpMode() {

        // --- 硬件映射与初始化 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aservo2.setDirection(Servo.Direction.REVERSE);
        iservo2.setDirection(Servo.Direction.REVERSE);

        // 初始状态下防溢出舵机闭合[cite: 1]
        aservo1.setPosition(0.4);
        aservo2.setPosition(0.4);

        telemetry.addLine("自动化射击控制系统就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. 档位设定 (D-pad) ---[cite: 1]
            if (gamepad1.dpad_up) {
                targetRPM = FULL_SPEED_RPM; iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                targetRPM = FULL_SPEED_RPM; iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                targetRPM = FULL_SPEED_RPM; iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                targetRPM = FULL_SPEED_RPM * 0.75; iCurrentPosition = 0.0;
            }

            // --- 5. 射击电机闭环控制 ---[cite: 2]
            double velocityCommand = 0;
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1;

            if (isTriggerPressed) {
                velocityCommand = (targetRPM * TICKS_PER_REV) / 60.0;
            }
            s1.setVelocity(velocityCommand);
            s2.setVelocity(velocityCommand);

            // --- 6. 转速监控与防溢出自动化 [核心改动] ---[cite: 2]
            double actualRPM1 = (s1.getVelocity() / TICKS_PER_REV) * 60.0;
            double actualRPM2 = (s2.getVelocity() / TICKS_PER_REV) * 60.0;

            // 判定逻辑：必须正在按 RT，且两个电机的转速都进入了误差允许范围[cite: 2]
            boolean speedReady = (targetRPM > 0) &&
                    (Math.abs(actualRPM1 - targetRPM) < RPM_TOLERANCE) &&
                    (Math.abs(actualRPM2 - targetRPM) < RPM_TOLERANCE);

            if (isTriggerPressed && speedReady) {
                // 自动开启：速度达标，放球射击[cite: 1]
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                // 自动闭合：速度不够或未按射击键，拦截球[cite: 1]
                aservo1.setPosition(0.4);
                aservo2.setPosition(0.4);
            }

            // --- 7. 角度调节与数据反馈 ---
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0, 1.0);
            iservo1.setPosition(iCurrentPosition);
            iservo2.setPosition(iCurrentPosition);

            telemetry.addData(">> 目标", "%.1f RPM", targetRPM);
            telemetry.addData(">> 实时 S1", "%.1f RPM", actualRPM1);
            telemetry.addData(">> 状态", speedReady ? "READY (已开启放行)" : "WAITING (挡板闭合)");
            telemetry.update();
        }
    }
}