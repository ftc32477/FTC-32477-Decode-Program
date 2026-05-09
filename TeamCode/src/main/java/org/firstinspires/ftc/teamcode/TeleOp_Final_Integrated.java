package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp_Final_Integrated_V2", group = "Test")
public class TeleOp_Final_Integrated extends LinearOpMode {

    // --- 1. 定义硬件对象 ---
    // 使用 DcMotorEx 以便调用 getVelocity() 获取速度[cite: 1]
    private DcMotorEx s1, s2;
    private Servo aservo1, aservo2;
    private Servo iservo1, iservo2;

    // --- 2. 状态变量 ---
    private double shootSpeed = 1.0;       // 射击功率变量[cite: 1]
    private double aCurrentPosition = 0.4; // Angle系列默认位置[cite: 1]
    private double iCurrentPosition = 0.0; // Inverted系列默认位置[cite: 1]

    // 假设使用 goBILDA 5203 系列电机 (常见的 19.2:1 减速比，约 537.7 ticks每转)
    // 如果你发现 RPM 数值不对，可以根据实际电机型号修改此数值
    private final double TICKS_PER_REV = 537.7;

    private final double ANGLE_INCREMENT = 20.0 / 180.0; // X/Y微调步进[cite: 1]
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // --- 3. 硬件映射与初始化 ---
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");

        // 电机初始化：重置编码器并启用编码器模式
        s1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 设置方向[cite: 1]
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        aservo1 = hardwareMap.get(Servo.class, "aservo1");
        aservo2 = hardwareMap.get(Servo.class, "aservo2");
        iservo1 = hardwareMap.get(Servo.class, "iservo1");
        iservo2 = hardwareMap.get(Servo.class, "iservo2");

        // 舵机方向与初始位置[cite: 1]
        aservo2.setDirection(Servo.Direction.REVERSE);
        iservo2.setDirection(Servo.Direction.REVERSE);

        aservo1.setPosition(aCurrentPosition);
        aservo2.setPosition(aCurrentPosition);
        iservo1.setPosition(iCurrentPosition);
        iservo2.setPosition(iCurrentPosition);

        telemetry.addLine("系统已就绪 | 编码器已激活");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. 档位切换逻辑 (左侧方向键)[cite: 1] ---
            if (gamepad1.dpad_up) {
                shootSpeed = 1.0; iCurrentPosition = 1.0;
            } else if (gamepad1.dpad_right) {
                shootSpeed = 1.0; iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_left) {
                shootSpeed = 1.0; iCurrentPosition = 0.0;
            } else if (gamepad1.dpad_down) {
                shootSpeed = 0.75; iCurrentPosition = 0.0;
            }

            // --- 5. 电机射击控制 (RT)[cite: 1] ---
            if (gamepad1.right_trigger > 0.1) {
                s1.setPower(shootSpeed);
                s2.setPower(shootSpeed);
            } else {
                s1.setPower(0);
                s2.setPower(0);
            }

            // --- 6. 速度测量 (关键新增部分) ---
            // 获取每秒脉冲数
            double vel1 = s1.getVelocity();
            double vel2 = s2.getVelocity();
            // 计算每分钟转数 (RPM)
            double rpm1 = (vel1 * 60.0) / TICKS_PER_REV;
            double rpm2 = (vel2 * 60.0) / TICKS_PER_REV;

            // --- 7. 舵机微调与覆盖逻辑[cite: 1] ---
            if (gamepad1.x && !lastX) iCurrentPosition += ANGLE_INCREMENT;
            if (gamepad1.y && !lastY) iCurrentPosition -= ANGLE_INCREMENT;
            iCurrentPosition = Range.clip(iCurrentPosition, 0.0, 1.0);

            iservo1.setPosition(iCurrentPosition);
            iservo2.setPosition(iCurrentPosition);

            if (gamepad1.right_bumper) {
                aservo1.setPosition(0.0);
                aservo2.setPosition(0.0);
            } else {
                aservo1.setPosition(aCurrentPosition);
                aservo2.setPosition(aCurrentPosition);
            }

            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // --- 8. 数据反馈 (包含实时转速) ---
            telemetry.addData("1. 目标功率", "%.2f", shootSpeed);
            telemetry.addData("2. 实测转速1", "%.0f RPM (%.0f ticks/s)", rpm1, vel1);
            telemetry.addData("3. 实测转速2", "%.0f RPM (%.0f ticks/s)", rpm2, vel2);
            telemetry.addData("4. iservo 目标位置", "%.2f", iCurrentPosition);
            telemetry.addData("5. aservo 状态", gamepad1.right_bumper ? "RB 强制归零" : "默认中位");

            // 如果两个电机转速差太大，给一个视觉警告
            if (Math.abs(rpm1 - rpm2) > 200) {
                telemetry.addLine("⚠️ 警告：双轮转速偏差较大！");
            }

            telemetry.update();
        }
    }
}