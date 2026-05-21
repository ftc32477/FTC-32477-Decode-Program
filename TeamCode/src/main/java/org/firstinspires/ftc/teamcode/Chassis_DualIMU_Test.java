package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 专有底盘测试程序 - 协同闭环版
 * 适用：纯有头模式测试、纯底层动力学等比例协同、软件级零点重置
 */
@TeleOp(name = "Chassis_DualIMU_Test", group = "Test")
public class Chassis_DualIMU_Test extends LinearOpMode {

    // ========== 1. 硬件对象与常数定义 ==========
    private DcMotorEx lf, rf, lb, rb;
    private IMU hubImu;
    private GoBildaPinpointDriver ppointOdo;

    // 操纵杆死区
    private static final double JOYSTICK_DEADZONE = 0.1;

    // 底盘电机专用高稳定PIDF参数 (来自一代底盘经典参数)
    private static final PIDFCoefficients CHASSIS_PIDF = new PIDFCoefficients(15.0, 3.0, 0.5, 12.0);

    // 双 IMU 融合权重
    private static final double WEIGHT_HUB_IMU = 0.3;
    private static final double WEIGHT_PINPOINT_IMU = 0.7;

    // 导航角度变量（软件零点控制）
    private double fusedHeading = 0.0;
    private double softwareYawOffset = 0.0; // 软件级绝对偏置量，彻底解决底层重置失效问题
    private boolean isFirstLoop = true;     // 第一帧标志

    // 历史编码器记录（用于等比例增量分析）
    private int lastLfPos, lastRfPos, lastLbPos, lastRbPos;

    @Override
    public void runOpMode() {
        // ========== 2. 硬件映射与初始化 ==========

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // 【严格保持不变】完全符合你车体实际的电机方向定义
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // 初始化底盘电机模式
        setupChassisMotor(lf);
        setupChassisMotor(rf);
        setupChassisMotor(lb);
        setupChassisMotor(rb);

        // Control Hub 内部 IMU 初始化
        hubImu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        hubImu.initialize(imuParameters);

        // Pinpoint 里程计物理配置
        ppointOdo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        ppointOdo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ppointOdo.setOffsets(0.0, -160.0);
        ppointOdo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        ppointOdo.resetPosAndIMU();
        softwareYawOffset = 0.0;
        isFirstLoop = true;

        telemetry.addLine("32477 底盘系统：软件零点对齐与刚性动力解算版就绪");
        telemetry.update();

        waitForStart();

        // 记录进入运行时的初始编码器位置
        lastLfPos = lf.getCurrentPosition();
        lastRfPos = rf.getCurrentPosition();
        lastLbPos = lb.getCurrentPosition();
        lastRbPos = rb.getCurrentPosition();

        while (opModeIsActive()) {
            // ========== 3. 导航数据更新与软件零点校准 ==========
            ppointOdo.update();

            double rawHubYaw = hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rawOdoYaw = Math.toDegrees(ppointOdo.getHeading());

            double deltaYaw = normalizeAngle(rawHubYaw - rawOdoYaw);
            fusedHeading = normalizeAngle(rawOdoYaw + WEIGHT_HUB_IMU * deltaYaw);

            // 【终极修正】如果底层重置不生效，在进入 loop 的第一帧自动在软件层面将当前融合角锁定为 0 点
            if (isFirstLoop) {
                softwareYawOffset = -fusedHeading;
                isFirstLoop = false;
            }

            double currentHeading = getFinalHeading();

            // ========== 4. 人机交互控制逻辑 ==========

            double drive = -gamepad1.left_stick_y;  // 前后
            double strafe = gamepad1.left_stick_x;  // 左右平移
            double turn = gamepad1.right_stick_x;    // 旋转

            // 死区判定
            if (Math.abs(drive) < JOYSTICK_DEADZONE) drive = 0;
            if (Math.abs(strafe) < JOYSTICK_DEADZONE) strafe = 0;
            if (Math.abs(turn) < JOYSTICK_DEADZONE) turn = 0;

            // 软件级一键归零：直接计算偏置，完全不依赖硬件延迟
            if (gamepad1.dpad_up) {
                softwareYawOffset = -fusedHeading;
                gamepad1.rumble(200);
            }

            // ========== 5. 动力分配与输出 (严格遵循你的动力方程) ==========

            // 【严格保持不变】你的标准运动学解算方程
            double fL = drive + strafe + turn;
            double fR = drive - strafe - turn;
            double bL = drive - strafe + turn;
            double bR = drive + strafe - turn;

            // 刚性协同监测（选读部分：老程序员用来在后台观测物理打滑的逻辑）
            int curLf = lf.getCurrentPosition();
            int curRf = rf.getCurrentPosition();
            int curLb = lb.getCurrentPosition();
            int curRb = rb.getCurrentPosition();

            // 计算当前这一帧四个轮子的实际脉冲变化量
            int dLf = curLf - lastLfPos;
            int dRf = curRf - lastRfPos;
            int dLb = curLb - lastLbPos;
            int dRb = curRb - lastRbPos;

            lastLfPos = curLf;
            lastRfPos = curRf;
            lastLbPos = curLb;
            lastRbPos = curRb;

            // 归一化缩放处理，防止因多向复合动作导致数值超限
            double maxPower = Math.max(Math.abs(fL), Math.max(Math.abs(fR), Math.max(Math.abs(bL), Math.abs(bR))));
            if (maxPower > 1.0) {
                fL /= maxPower;
                fR /= maxPower;
                bL /= maxPower;
                bR /= maxPower;
            }

            // 直接输出至具备 PIDF 速度闭环的电机上，没有任何导航角强行干预操控感
            lf.setPower(fL);
            rf.setPower(fR);
            lb.setPower(bL);
            rb.setPower(bR);

            // ========== 6. 实时遥测数据监控 (Telemetry) ==========
            telemetry.addLine("== 32477 TeleOp Monitor ==");
            telemetry.addData("软件重置后航向 (Heading)", "%.2f °", currentHeading);
            telemetry.addData("Hub IMU 原始值", "%.2f °", rawHubYaw);
            telemetry.addData("Pinpoint IMU 原始值", "%.2f °", rawOdoYaw);
            telemetry.addLine("----------------------------------");
            telemetry.addLine("各轮单帧脉冲变化量(用于诊断偏转原因):");
            telemetry.addData("LF / RF 变化", "%d | %d", dLf, dRf);
            telemetry.addData("LB / RB 变化", "%d | %d", dLb, dRb);
            telemetry.addLine("----------------------------------");
            telemetry.addData("Odo 估计坐标 X", "%.1f cm", ppointOdo.getPosition().getX(DistanceUnit.CM));
            telemetry.addData("Odo 估计坐标 Y", "%.1f cm", ppointOdo.getPosition().getY(DistanceUnit.CM));
            telemetry.update();
        }

        // 退出时彻底清空底盘动力
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    private void setupChassisMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CHASSIS_PIDF);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * 获取考虑软件重置偏移量后的最终航向角
     */
    private double getFinalHeading() {
        return normalizeAngle(fusedHeading + softwareYawOffset);
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}