package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    // 声明所有电机
    public DcMotorEx lf, rf, lb, rb;
    public DcMotorEx intake, s1, s2, loader; // 新增 loader

    // 声明舵机
    public Servo lift;
    public Servo center;

    // 声明陀螺仪
    public IMU imu;

    public void init(HardwareMap hwMap) {
        // --- 硬件映射 ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");

        intake = hwMap.get(DcMotorEx.class, "intake");
        loader = hwMap.get(DcMotorEx.class, "loader"); // 初始化 loader
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");

        lift = hwMap.get(Servo.class, "lift");
        center = hwMap.get(Servo.class, "center");
        imu = hwMap.get(IMU.class, "imu");

        // --- 电机方向 ---
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        s1.setDirection(DcMotor.Direction.FORWARD);
        s2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        loader.setDirection(DcMotor.Direction.REVERSE);

        // --- 刹车模式 ---
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // loader 需要精准停止

        // --- 陀螺仪初始化 ---
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // --- 舵机初始位置 ---
        lift.setPosition(0.5);
        // 确保 360 舵机初始状态为停止 (0.5 通常是停止信号)
        center.setPosition(0.5);
    }
}