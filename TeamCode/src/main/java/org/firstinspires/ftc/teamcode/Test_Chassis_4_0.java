package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 2025-2026 'Decode' 赛季底盘调试程序
 * 映射逻辑：摇杆前推 -> 机器人左移
 * 映射逻辑：摇杆右推 -> 机器人前移
 */
@TeleOp(name = "Test_Chassis_4_0", group = "Debug")
public class Test_Chassis_4_0 extends LinearOpMode {

    // 电机定义
    private DcMotor lf, rf, lb, rb;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 1. 硬件映射 (匹配你的配置名称)
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // 2. 设置电机方向
        // 沿用上一代配置：右侧反向
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // 3. 设置零功率行为为刹车
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "调试程序已就绪 - 摇杆前推应左移");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // 获取原始摇杆输入
            // gamepad1.left_stick_y: 前推为负, 后拉为正
            // gamepad1.left_stick_x: 左推为负, 右推为正
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            /**
             * 坐标轴重映射逻辑：
             * 我们希望：rawY (前) 触发机器人向左平移 (Strafe -1)
             * 我们希望：rawX (右) 触发机器人向前行驶 (Drive +1)
             */
            double drive = rawX;      // 摇杆左右控制前后
            double strafe = -rawY;    // 摇杆前后控制左右 (前推rawY为正，赋予strafe为负即左移)

            // 麦克纳姆轮运动学计算
            double fL = drive + strafe + turn;
            double fR = drive - strafe - turn;
            double bL = drive - strafe + turn;
            double bR = drive + strafe - turn;

            // 归一化功率，防止超过 1.0
            double max = Math.max(Math.abs(fL), Math.max(Math.abs(fR),
                    Math.max(Math.abs(bL), Math.abs(bR))));
            if (max > 1.0) {
                fL /= max; fR /= max; bL /= max; bR /= max;
            }

            // 输出到电机
            lf.setPower(fL);
            rf.setPower(fR);
            lb.setPower(bL);
            rb.setPower(bR);

            // 遥测数据反馈
            telemetry.addData("Status", "运行中");
            telemetry.addData("输入 (Raw)", "Y:%.2f, X:%.2f", rawY, rawX);
            telemetry.addData("逻辑 (Mapped)", "Drive:%.2f, Strafe:%.2f", drive, strafe);
            telemetry.addData("电机功率", "LF:%.2f, RF:%.2f", fL, fR);
            telemetry.addData("电机功率", "LB:%.2f, RB:%.2f", bL, bR);
            telemetry.update();
        }

        // 停止电机
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}