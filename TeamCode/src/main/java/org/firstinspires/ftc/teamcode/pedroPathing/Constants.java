package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
// 核心修复：使用 Pedro Pathing 自己的 PID 控制类，而非 Qualcomm SDK 的
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ==============================================================================
    // 1. 跟随器核心参数 - 【准备进行调参测试】
    // ==============================================================================
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5)

            // --- A. 零动力摩擦力 (需跑 ZeroPowerTuner) ---
            .forwardZeroPowerAcceleration(-43)
            .lateralZeroPowerAcceleration(-70)

            // --- B. 核心 PIDF 调参 (使用 Pedro Pathing 原生类) ---
            // Translational/Heading 使用标准 PIDF (4参: P, I, D, F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.03, 0.02))

            // Drive 使用 FilteredPIDF (5参: P, I, D, F, Filter)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0, 0.8))

            // --- C. 向心力系数 ---
            .centripetalScaling(0.0); // TODO: 跑 CentripetalForceTuner 后填入

    // ==============================================================================
    // 2. 底盘硬件映射 (需根据实际物理极限填入)
    // ==============================================================================
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // 👇 必须填入跑 VelocityTuner 测得的真实值
            .xVelocity(69) // 前向极限速度
            .yVelocity(48); // 横移极限速度

    // ==============================================================================
    // 3. 物理极限约束
    // ==============================================================================
    public static PathConstraints pathConstraints = new PathConstraints(
            80.0,  // 最大线速度
            60.0,  // 最大线加速度
            3.0,   // 最大角速度
            2.0    // 最大角加速度
    );

    // ==============================================================================
    // 4. 定位器常数 (已校准)
    // ==============================================================================
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(94)
            .strafePodX(18)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ==============================================================================
    // 5. 组装工厂
    // ==============================================================================
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}