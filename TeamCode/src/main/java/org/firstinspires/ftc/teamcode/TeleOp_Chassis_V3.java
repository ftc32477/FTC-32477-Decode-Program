package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 32477 三代车底盘最终修正版 - V3.3
 * 所属学校：北京十一实验中学[cite: 1]
 * 赛季：2025-2026 “Decode”[cite: 1]
 *
 * 修正内容：
 * 1. 电机方向：左侧(lf, lb) REVERSE，右侧(rf, rb) FORWARD[cite: 4]
 * 2. Hub方位：Logo向左 (LEFT)，USB向前 (FORWARD)
 * 3. 坐标修正：调整 Pinpoint 编码器方向以实现向前为正
 * 4. 传感器重置：初始化时 IMU 和 Odo 自动归零
 */
@TeleOp(name = "32477_V3_Final_Direction_Test", group = "Production")
public class TeleOp_Chassis_V3 extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private IMU imu;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // --- 1. 硬件映射 ---
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- 2. 电机方向修正 (根据最新测试结果) ---
        // 32477 V3 逻辑：左反右正[cite: 4]
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // 设置制动
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 3. IMU 初始化 (Logo向左, USB向前) ---
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw(); // 初始航向重置为0

        // --- 4. Pinpoint 里程计配置[cite: 7] ---
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(14.0, 92.0); // 沿用二代车稳定参数[cite: 7]

        /*
         * 坐标系修正逻辑：
         * 原读数向前为负，说明纵向编码器方向需要反转。
         * 如果纵向反转后仍不准，请将下方的 FORWARD 改为 REVERSE。
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU(); // 全局坐标与航向清零[cite: 7]

        telemetry.addLine(">> 32477 V3 最终修正版已准备就绪");
        telemetry.addLine(">> 坐标系：向前为X+，向左为Y+");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update(); // 必须每轮循环更新[cite: 7]

            // --- 5. 底盘控制 (机器人坐标系) ---
            double drive = -gamepad1.left_stick_y; // 向上推是正向移动
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // 麦克纳姆轮经典算法
            double pLF = drive + strafe + turn;
            double pRF = drive - strafe - turn;
            double pLB = drive - strafe + turn;
            double pRB = drive + strafe - turn;

            // 功率归一化处理
            double max = Math.max(Math.abs(pLF), Math.max(Math.abs(pRF),
                    Math.max(Math.abs(pLB), Math.abs(pRB))));
            if (max > 1.0) {
                pLF /= max; pRF /= max; pLB /= max; pRB /= max;
            }

            lf.setPower(pLF);
            rf.setPower(pRF);
            lb.setPower(pLB);
            rb.setPower(pRB);

            // --- 6. 调试数据反馈 ---
            Pose2D pos = odo.getPosition();
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Status", "V3 Production Ready");
            telemetry.addLine("---- 位置坐标 (mm) ----");
            telemetry.addData("X (向前应该增加)", "%.1f", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y (向左应该增加)", "%.1f", pos.getY(DistanceUnit.MM));
            telemetry.addLine("---- 航向角 (Deg) ----");
            telemetry.addData("Pinpoint Heading", "%.2f°", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Hub IMU Yaw", "%.2f°", yaw);
            telemetry.update();
        }
    }
}