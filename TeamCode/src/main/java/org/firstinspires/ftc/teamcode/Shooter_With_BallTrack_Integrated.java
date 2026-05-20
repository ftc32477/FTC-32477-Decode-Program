package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter_With_BallTrack_Integrated", group = "Production")
public class Shooter_With_BallTrack_Integrated extends LinearOpMode {

    // ========== 1. 硬件对象 ==========
    // 发射电机与舵机
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2;
    private Servo iservo1, iservo2;

    // 【新增】球道与吸球电机
    private DcMotor intake = null;
    private DcMotor load = null;

    // ========== 2. 常数定义 ==========
    // 针对 6000RPM 电机修正比率
    private final double TICKS_PER_REV = 28.0;
    private final double P = 15.0, I = 0.0, D = 1.5, F = 17.0;
    private final double RPM_TOLERANCE = 100.0;

    // 怠速提升至 1400 RPM，获取极速启动响应
    private final double IDLE_RPM = 1400.0;

    // 混合算法切换阈值
    private final double BANGBANG_THRESHOLD = 80.0;

    // ========== 3. 状态变量 ==========
    private double targetRPM = 0;
    private double iCurrentPosition = 0.0; // 俯仰初始值回归 0.0

    @Override
    public void runOpMode() {

        // --- 硬件映射 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        // 【新增】球道硬件映射
        intake = hardwareMap.get(DcMotor.class, "intake");
        load = hardwareMap.get(DcMotor.class, "load");

        // --- 电机方向修正 ---
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        // 【新增】球道方向设置
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);

        // --- 舵机方向设定 ---
        aservo2.setDirection(Servo.Direction.REVERSE);
        iservo2.setDirection(Servo.Direction.REVERSE);

        // 重置并启用内置 PIDF 速度闭环
        s1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 写入调试好的高参数 PIDF 核心系数
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        // 【新增】球道停止行为：设置为刹车模式，防止惯性转动
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始挡板物理闭合 (0.4)
        aservo1.setPosition(0.4);
        aservo2.setPosition(0.4);

        telemetry.addLine("32477: 整体集成系统已准备就绪");
        telemetry.addLine(">> 操控指南:");
        telemetry.addLine("   - 方向键: 切换射击档位");
        telemetry.addLine("   - Right Trigger: 激活射击/怠速机制");
        telemetry.addLine("   - X / Y: 微调俯仰角度");
        telemetry.addLine("   - 按 A: 单独吸球 (Intake Only)");
        telemetry.addLine("   - 按 B: 联动装填 (Intake + Load)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. 射击档位设定 ---
            if (gamepad1.dpad_up) {
                targetRPM = 2150.0;
                iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                targetRPM = 2150.0;
                iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                targetRPM = 2150.0;
                iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                targetRPM = 1600.0;
                iCurrentPosition = 0.0;
            }

            // --- 5. 目标速度逻辑状态切换 ---
            double currentTargetSpeed = 0;
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1;

            if (isTriggerPressed) {
                currentTargetSpeed = targetRPM;
            } else if (targetRPM > 0) {
                currentTargetSpeed = IDLE_RPM;
            } else {
                currentTargetSpeed = 0;
            }

            // --- 6. RPM 实时数据获取 ---
            double actualRPM1 = (s1.getVelocity() / TICKS_PER_REV) * 60.0;
            double actualRPM2 = (s2.getVelocity() / TICKS_PER_REV) * 60.0;

            // --- 7. PIDF + Bang-Bang 混合动态控速核心 ---
            if (currentTargetSpeed > 100) {
                double targetTicksPerSec = (currentTargetSpeed / 60.0) * TICKS_PER_REV;

                // 计算当前转速与目标的绝对差值
                double error1 = currentTargetSpeed - actualRPM1;
                double error2 = currentTargetSpeed - actualRPM2;

                // --- S1 电机混合控制 ---
                if (error1 >= BANGBANG_THRESHOLD) {
                    s1.setPower(1.0); // 掉速超过80转，立刻触发 Bang-Bang 砸满电压强补
                } else {
                    s1.setVelocity(targetTicksPerSec); // 误差进入80转内，切回 PIDF 丝滑精准控速
                }

                // --- S2 电机混合控制 ---
                if (error2 >= BANGBANG_THRESHOLD) {
                    s2.setPower(1.0); // 掉速超过80转，立刻触发 Bang-Bang 砸满电压强补
                } else {
                    s2.setVelocity(targetTicksPerSec); // 误差进入80转内，切回 PIDF 丝滑精准控速
                }
            } else {
                // 未选定任何状态或处于 0 速，彻底关停电机
                s1.setVelocity(0);
                s2.setVelocity(0);
                s1.setPower(0);
                s2.setPower(0);
            }

            // --- 8. RPM 监控与挡板自动化判定 ---
            boolean speedReady = isTriggerPressed && (targetRPM > 500) &&
                    (Math.abs(actualRPM1 - targetRPM) < RPM_TOLERANCE) &&
                    (Math.abs(actualRPM2 - targetRPM) < RPM_TOLERANCE);

            if (speedReady) {
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                aservo1.setPosition(0.4);
                aservo2.setPosition(0.4);
            }

            // --- 9. 俯仰核心逻辑 ---
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0, 1.0);

            iservo1.setPosition(iCurrentPosition);
            iservo2.setPosition(iCurrentPosition);

            // --- 10. 【新增】球道与吸球控制逻辑 ---
            String ballTrackStatus;
            if (gamepad1.b) {
                // 优先检测 B 键：两者同时转动
                intake.setPower(0.9);
                load.setPower(0.9);
                ballTrackStatus = "Intake & Load Running";
            } else if (gamepad1.a) {
                // 检测 A 键：仅 intake 转动
                intake.setPower(0.9);
                load.setPower(0.0);
                ballTrackStatus = "Intake Only";
            } else {
                // 无按键：全部停止
                intake.setPower(0.0);
                load.setPower(0.0);
                ballTrackStatus = "Idle";
            }

            // --- 11. 详细综合数据监控 ---
            telemetry.addData("Selected Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current Run Speed", "%.0f RPM", currentTargetSpeed);
            telemetry.addData("S1 Actual RPM", "%.1f", actualRPM1);
            telemetry.addData("S2 Actual RPM", "%.1f", actualRPM2);

            // 监控当前每个电机的控制状态
            telemetry.addData("S1 Control Mode", (currentTargetSpeed - actualRPM1 >= BANGBANG_THRESHOLD) ? "BANG-BANG (MAX)" : "PIDF (STEADY)");
            telemetry.addData("S2 Control Mode", (currentTargetSpeed - actualRPM2 >= BANGBANG_THRESHOLD) ? "BANG-BANG (MAX)" : "PIDF (STEADY)");
            telemetry.addData("Ready Status", speedReady ? "READY TO SHOOT" : "NOT READY / IDLE");

            telemetry.addLine("----------");
            telemetry.addData("Pitch Position", "%.3f", iCurrentPosition);
            // 【新增】球道监控数据
            telemetry.addData("BallTrack Status", ballTrackStatus);

            telemetry.update();
        }
    }
}