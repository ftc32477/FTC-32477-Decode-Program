package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter_Auto_Fire", group = "Production")
public class Shooter_Auto_Fire extends LinearOpMode {

    // ========== 1. 硬件对象 ==========
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2;
    private Servo iservo1, iservo2;

    // ========== 2. 常数定义 ==========
    // 针对 6000RPM 电机修正比率
    private final double TICKS_PER_REV = 28.0;
    private final double P = 6.2, I = 0.0, D = 1.5, F = 17.5;
    private final double RPM_TOLERANCE = 150.0;

    // ========== 3. 状态变量 ==========
    private double targetRPM = 0;
    // 【修正】俯仰初始值回归 0.0 (模仿原始程序)
    private double iCurrentPosition = 0.0;

    @Override
    public void runOpMode() {

        // --- 硬件映射 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        // --- 电机方向修正 ---
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        // --- 舵机方向设定 ---
        // 挡板使用 REVERSE，而俯仰舵机我们通过 setPosition 逻辑来实现“一侧0->1，一侧1->0”
        aservo2.setDirection(Servo.Direction.REVERSE);
        // 移除 iservo2 的 setDirection，避免逻辑冲突

        // 配置电机模式
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 初始挡板物理闭合 (0.4)
        aservo1.setPosition(0.4);
        aservo2.setPosition(0.4);

        telemetry.addLine("32477: 系统已准备就绪 (RPM监控+俯仰0.0初始)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. 档位设定 (完全模仿原逻辑) ---
            if (gamepad1.dpad_up) {
                targetRPM = 2000.0;
                iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                targetRPM = 2000.0;
                iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                targetRPM = 2000.0;
                iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                targetRPM = 1600.0;
                iCurrentPosition = 0.0;
            }

            // --- 5. 射击电机动力输出 ---
            double velocityCommand = 0;
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1;

            if (isTriggerPressed) {
                velocityCommand = (targetRPM * TICKS_PER_REV) / 60.0;
            }
            s1.setVelocity(velocityCommand);
            s2.setVelocity(velocityCommand);

            // --- 6. RPM 监控与判定 ---
            double actualRPM1 = (s1.getVelocity() / TICKS_PER_REV) * 60.0;
            double actualRPM2 = (s2.getVelocity() / TICKS_PER_REV) * 60.0;

            boolean speedReady = (targetRPM > 500) &&
                    (Math.abs(actualRPM1 - targetRPM) < RPM_TOLERANCE) &&
                    (Math.abs(actualRPM2 - targetRPM) < RPM_TOLERANCE);

            // 自动化挡板
            if (isTriggerPressed && speedReady) {
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                aservo1.setPosition(0.4);
                aservo2.setPosition(0.4);
            }

            // --- 7. 【俯仰核心】左侧 0->1，右侧 1->0 镜像逻辑 ---
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0, 1.0);

            iservo1.setPosition(iCurrentPosition);
            iservo2.setPosition(1.0 - iCurrentPosition); // 数学镜像防止角力卡死

            // --- 8. 【回归】详细数据监控 ---
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("S1 Actual RPM", "%.1f", actualRPM1);
            telemetry.addData("S2 Actual RPM", "%.1f", actualRPM2);
            telemetry.addData("Ready Status", speedReady ? "YES" : "NO");
            telemetry.addLine("----------");
            telemetry.addData("Pitch Position", "%.3f", iCurrentPosition);
            telemetry.addData("Servo1 Pos", "%.3f", iCurrentPosition);
            telemetry.addData("Servo2 Pos", "%.3f", 1.0 - iCurrentPosition);
            telemetry.update();
        }
    }
}