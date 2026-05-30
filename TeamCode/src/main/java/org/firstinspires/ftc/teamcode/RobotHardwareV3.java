package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardwareV3 {

    // ========== 1. 硬件对象声明 ==========
    // 底盘动力轮
    public DcMotorEx lf = null;
    public DcMotorEx rf = null;
    public DcMotorEx lb = null;
    public DcMotorEx rb = null;

    // 传感器及里程计
    public IMU hubImu = null;
    public GoBildaPinpointDriver ppointOdo = null;

    // 发射机构 (Shooter)
    public DcMotorEx s1 = null;
    public DcMotorEx s2 = null;
    public Servo aservo1 = null;
    public Servo aservo2 = null;
    public Servo iservo1 = null;
    public Servo iservo2 = null;

    // 拾取与球道机构 (BallTrack)
    public DcMotor intake = null;
    public DcMotor load = null;

    // ========== 2. 核心物理常数定义 ==========
    public final double SHOOTER_TICKS_PER_REV = 28.0;

    // 内闭环 PIDF 参数
    public final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(12.0, 1.0, 5.0, 18.0);
    public final PIDFCoefficients CHASSIS_PIDF = new PIDFCoefficients(12.5, 3.0, 0.5, 11.5);

    private HardwareMap hwMap = null;

    public RobotHardwareV3() {}

    /* 初始化硬件接口 */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // --- 底盘动力总成映射 ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");

        // 麦轮正反向镜像修正
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        setupChassisMotor(lf);
        setupChassisMotor(rf);
        setupChassisMotor(lb);
        setupChassisMotor(rb);

        // 初始化内置 IMU
        hubImu = hwMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        hubImu.initialize(imuParameters);
        hubImu.resetYaw();

        // 初始化 Pinpoint 里程计
        ppointOdo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        ppointOdo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ppointOdo.setOffsets(-74.0, -136.0);
        ppointOdo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ppointOdo.resetPosAndIMU();

        // --- 射击与吸球球道机构映射 ---
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");
        aservo1 = hwMap.get(Servo.class, "aservo1");
        aservo2 = hwMap.get(Servo.class, "aservo2");
        iservo1 = hwMap.get(Servo.class, "iservo1");
        iservo2 = hwMap.get(Servo.class, "iservo2");
        intake = hwMap.get(DcMotor.class, "intake");
        load = hwMap.get(DcMotor.class, "load");

        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        // 【修改点1】将 load 电机物理反转设定
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.FORWARD); // 改变默认方向以顺应倒转需求

        s1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s1.setPower(0);
        s2.setPower(0);
        intake.setPower(0);
        load.setPower(0);

        aservo1.setDirection(Servo.Direction.FORWARD);
        aservo2.setDirection(Servo.Direction.REVERSE);
        aservo1.setPosition(0.4);
        aservo2.setPosition(0.4);
    }

    private void setupChassisMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CHASSIS_PIDF);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
}