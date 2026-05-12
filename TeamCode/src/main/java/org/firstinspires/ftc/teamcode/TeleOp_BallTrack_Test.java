package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * 32477 三代车球道专项测试程序 (修正版)
 * 逻辑：
 * 1. 按下 A：仅 intake 转动
 * 2. 按下 B：intake 和 load 同时转动
 * 3. 方向：load 设置为 REVERSE
 */
@TeleOp(name = "BallTrack_Test_V2", group = "Test")
public class TeleOp_BallTrack_Test extends LinearOpMode {

    private DcMotor intake = null;
    private DcMotor load = null;

    @Override
    public void runOpMode() {
        // 1. 硬件映射
        intake = hardwareMap.get(DcMotor.class, "intake");
        load = hardwareMap.get(DcMotor.class, "load");

        // 2. 方向设置：按照要求 load 设置为反向
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);

        // 3. 停止行为：设置为刹车模式，防止惯性转动
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine(">> 32477 程序就绪");
        telemetry.addLine(">> 按 A: 单独吸球 (Intake Only)");
        telemetry.addLine(">> 按 B: 联动装填 (Intake + Load)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                // 优先检测 B 键：两者同时转动
                intake.setPower(0.9);
                load.setPower(0.9);
                telemetry.addData("Status", "B Pressed: Intake & Load Running");
            } else if (gamepad1.a) {
                // 检测 A 键：仅 intake 转动
                intake.setPower(0.9);
                load.setPower(0.0);
                telemetry.addData("Status", "A Pressed: Intake Only");
            } else {
                // 无按键：全部停止
                intake.setPower(0.0);
                load.setPower(0.0);
                telemetry.addData("Status", "Idle");
            }

            telemetry.update();
        }
    }
}