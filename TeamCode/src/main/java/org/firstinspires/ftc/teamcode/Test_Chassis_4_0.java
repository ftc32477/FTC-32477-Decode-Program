package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 2025-2026 'Decode' 赛季 - 横置底盘调试程序 (修正版)
 * 硬件特征：电机轴指向机器人正前方
 * 修正逻辑：摇杆前推 -> 机器人直行；摇杆左右 -> 机器人平移
 */
@TeleOp(name = "Test_Chassis_4_0", group = "Debug")
public class Test_Chassis_4_0 extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 1. 硬件映射
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // 2. 方向设置 (保持原有配置)
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "横置底盘修正程序已就绪");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // 获取摇杆原始输入
            double drive = -gamepad1.left_stick_y;  // 前推为正
            double strafe = gamepad1.left_stick_x; // 右推为正
            double turn = gamepad1.right_stick_x;  // 旋转为正

            /**
             * 核心算法更改：
             * 根据反馈，原本的 drive(前推) 导致了右平移，而 strafe(左推) 导致了前进。
             * 我们重新映射分量，使：
             * - drive 为正时，所有电机产生“前进”所需的组合。
             * - strafe 为正时，所有电机产生“向右平移”所需的组合。
             */
            double fL = -drive - strafe - turn;
            double fR = -drive + strafe + turn;
            double bL = -drive + strafe - turn;
            double bR = -drive - strafe + turn;

            // 3. 归一化处理
            double max = Math.max(Math.abs(fL), Math.max(Math.abs(fR),
                    Math.max(Math.abs(bL), Math.abs(bR))));
            if (max > 1.0) {
                fL /= max; fR /= max; bL /= max; bR /= max;
            }

            // 4. 输出到电机
            lf.setPower(fL);
            rf.setPower(fR);
            lb.setPower(bL);
            rb.setPower(bR);

            // 遥测数据
            telemetry.addData("输入", "Drive:%.2f, Strafe:%.2f", drive, strafe);
            telemetry.addData("输出功率", "LF:%.2f, RF:%.2f, LB:%.2f, RB:%.2f", fL, fR, bL, bR);
            telemetry.update();
        }

        lf.setPower(0); rf.setPower(0); lb.setPower(0); rb.setPower(0);
    }
}