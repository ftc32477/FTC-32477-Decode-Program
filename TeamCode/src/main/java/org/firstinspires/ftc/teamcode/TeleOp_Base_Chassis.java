package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * 32477基础底盘程序 - 模仿2025-2026一代车
 * 仅保留左右摇杆控制底盘的能力
 */
@TeleOp(name = "TeleOp_Base_Chassis", group = "Production")
public class TeleOp_Base_Chassis extends LinearOpMode {

    // 定义四个底盘电机
    private DcMotorEx lf, rf, lb, rb;

    @Override
    public void runOpMode() {
        // 1. 硬件映射 (必须与一代车 Robot Configuration 一致)[cite: 8, 9]
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // 2. 设置电机方向
        // 麦克纳姆轮标准配置：右侧电机通常需要反转
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // 3. 设置零功率行为为刹车 (防止滑行)[cite: 8, 9]
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("32477 基础底盘初始化完成");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 获取摇杆输入 (加上负号是因为推摇杆向上时 Y 是负值)
            double drive = -gamepad1.left_stick_y;  // 前后
            double strafe = gamepad1.left_stick_x;  // 左右平移
            double turn = gamepad1.right_stick_x;   // 旋转

            // 4. 麦克纳姆轮动力分配算法[cite: 9]
            double fL = drive + strafe + turn;
            double fR = drive - strafe - turn;
            double bL = drive - strafe + turn;
            double bR = drive + strafe - turn;

            // 5. 归一化处理：确保电机功率不超过 1.0[cite: 9]
            double max = Math.max(Math.abs(fL), Math.max(Math.abs(fR),
                    Math.max(Math.abs(bL), Math.abs(bR))));
            if (max > 1.0) {
                fL /= max; fR /= max; bL /= max; bR /= max;
            }

            // 输出功率
            lf.setPower(fL);
            rf.setPower(fR);
            lb.setPower(bL);
            rb.setPower(bR);

            // 遥测数据监控
            telemetry.addData("Status", "Running");
            telemetry.addData("Drive/Strafe/Turn", "%.2f | %.2f | %.2f", drive, strafe, turn);
            telemetry.update();
        }
    }
}