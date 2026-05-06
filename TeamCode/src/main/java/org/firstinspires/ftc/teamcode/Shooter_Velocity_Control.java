package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter_Velocity_Control", group = "Test")
public class Shooter_Velocity_Control extends LinearOpMode {

    // --- 1. 硬件对象 ---
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2, iservo1, iservo2;

    // --- 2. 常数定义 (模仿 v4.0 风格) ---
    // 假设使用 goBILDA 5203 系列电机 (28 Ticks/Rev 是编码器原始值，19.2:1 减速后约为 537.7)
    // 注意：v4.0 源码中使用的是 28，通常对应的是电机轴直接测量
    final double SHOOTER_TICKS = 537.7;

    // PIDF 参数：这些参数决定了电机达到目标转速的速度和稳定性[cite: 2]
    // 如果电机晃动剧烈，调小 P；如果达不到速度，调大 F
    final double P = 15.0, I = 3.0, D = 1.0, F = 12.0;

    final double MAX_RPM = 175.0; // 用户要求的满速[cite: 2]
    final double ANGLE_INCREMENT = 0.005; // 俯仰角微调步长

    // --- 3. 状态变量 ---
    private double targetRPM = 0;
    private double iCurrentPosition = 0.5;

    @Override
    public void runOpMode() {
        // --- 硬件映射 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        // --- 电机配置 (模仿 v4.0 的闭环设置) ---
        s1.setDirection(DcMotor.Direction.REVERSE); // 维持你之前的方向设置
        s2.setDirection(DcMotor.Direction.FORWARD);

        // 应用 PIDF 系数[cite: 2]
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- 舵机配置 ---
        aservo2.setDirection(Servo.Direction.REVERSE);
        iservo2.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("速度闭环控制系统已就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 控制逻辑 1：速度设定 ---
            // 按下 RT 时，设定目标为满速 175 RPM[cite: 2]
            if (gamepad1.right_trigger > 0.1) {
                targetRPM = MAX_RPM;
            } else {
                targetRPM = 0;
            }

            // 将 RPM 转换为电机需要的 Velocity (Ticks/Second)
            // 公式：(RPM * TicksPerRev) / 60 秒
            double targetVelocity = (targetRPM * SHOOTER_TICKS) / 60.0;

            // 使用 setVelocity 而不是 setPower！这是闭环控制的核心[cite: 2]
            s1.setVelocity(targetVelocity);
            s2.setVelocity(targetVelocity);

            // --- 控制逻辑 2：角度微调 (模仿 v4.0 的平滑调节) ---
            if (gamepad1.x) {
                iCurrentPosition = Range.clip(iCurrentPosition + ANGLE_INCREMENT, 0, 1.0);
            } else if (gamepad1.y) {
                iCurrentPosition = Range.clip(iCurrentPosition - ANGLE_INCREMENT, 0, 1.0);
            }
            iservo1.setPosition(iCurrentPosition);
            iservo2.setPosition(iCurrentPosition);

            // --- 控制逻辑 3：防溢出保护 (RB) ---
            if (gamepad1.right_bumper) {
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                aservo1.setPosition(0.4);
                aservo2.setPosition(0.4);
            }

            // --- 遥测数据反馈 ---
            double currentRPM1 = (s1.getVelocity() / SHOOTER_TICKS) * 60.0;
            double currentRPM2 = (s2.getVelocity() / SHOOTER_TICKS) * 60.0;

            telemetry.addData("目标状态", targetRPM > 0 ? "发射 (175 RPM)" : "停止");
            telemetry.addData("实时转速 S1", "%.1f RPM", currentRPM1);
            telemetry.addData("实时转速 S2", "%.1f RPM", currentRPM2);
            telemetry.addData("俯仰角度", "%.3f", iCurrentPosition);

            // 简单的达标判断[cite: 2]
            if (targetRPM > 0 && Math.abs(currentRPM1 - targetRPM) < 5) {
                telemetry.addLine(">> 转速已稳定，准许发射！ <<");
            }

            telemetry.update();
        }
    }
}