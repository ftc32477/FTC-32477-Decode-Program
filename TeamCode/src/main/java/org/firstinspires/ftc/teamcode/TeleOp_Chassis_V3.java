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
 * 32477 三代车底盘基础测试程序 - V3.2
 * 功能：
 * 1. 电机全 Reverse 修正运行方向
 * 2. 纯机器人坐标系控制（已去除无头模式）
 * 3. 实时监控二代车迭代来的 Pinpoint 定位与 IMU 原始数值
 */
@TeleOp(name = "32477_V3_Direction_Sensor_Test", group = "Test")
public class TeleOp_Chassis_V3 extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private IMU imu;
    private GoBildaPinpointDriver odo; // 官方 Pinpoint 支持库

    @Override
    public void runOpMode() {
        // --- 1. 硬件映射 ---
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // --- 2. 方向修正：根据要求所有电机设置为 REVERSE ---
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // 设置制动模式以方便精准停靠
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 3. IMU 初始化 ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // --- 4. Pinpoint (Odo) 初始化 ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // 继承自二代车 TeleOp_All_4_0 的设置[cite: 7]
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(0.0, -160.0);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addLine("方向已反转，传感器监控就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 更新定位系统
            odo.update();

            // --- 5. 获取传感器数值 ---
            Pose2D pos = odo.getPosition(); // 获取 Pinpoint 坐标和航向
            double hubYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // --- 6. 基础底盘控制 (Robot Centric) ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double pLF = drive + strafe + turn;
            double pRF = drive - strafe - turn;
            double pLB = drive - strafe + turn;
            double pRB = drive + strafe - turn;

            // 归一化限制
            double max = Math.max(Math.abs(pLF), Math.max(Math.abs(pRF),
                    Math.max(Math.abs(pLB), Math.abs(pRB))));
            if (max > 1.0) {
                pLF /= max; pRF /= max; pLB /= max; pRB /= max;
            }

            lf.setPower(pLF);
            rf.setPower(pRF);
            lb.setPower(pLB);
            rb.setPower(pRB);

            // --- 7. 数据显示 (重点监控项目) ---
            telemetry.addLine("== 传感器数值监控 ==");
            telemetry.addData("Hub IMU Yaw", "%.2f°", hubYaw);
            telemetry.addData("Pinpoint X (mm)", "%.1f", pos.getX(DistanceUnit.MM));
            telemetry.addData("Pinpoint Y (mm)", "%.1f", pos.getY(DistanceUnit.MM));
            telemetry.addData("Pinpoint Heading", "%.2f°", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("------------------");
            telemetry.addData("电机功率", "LF:%.2f RF:%.2f", pLF, pRF);
            telemetry.update();
        }
    }
}