package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    // 声明所有电机
    public DcMotorEx lf, rf, lb, rb;
    public DcMotorEx intake, s1, s2, loader;

    // 声明舵机
    public Servo lift;
    public Servo center;
    public Servo loaderservo;

    // 声明陀螺仪
    public IMU imu;

    public void init(HardwareMap hwMap) {
        // --- 硬件映射 ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");

        intake = hwMap.get(DcMotorEx.class, "intake");
        loader = hwMap.get(DcMotorEx.class, "loader");
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");

        lift = hwMap.get(Servo.class, "lift");
        center = hwMap.get(Servo.class, "center");
        loaderservo = hwMap.get(Servo.class, "loaderservo");

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
        loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 陀螺仪初始化 ---
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // --- loaderservo 10秒初始化归位逻辑 ---
        // 将180度舵机推至逆时针物理极限位置 (0.0)
        ElapsedTime initTimer = new ElapsedTime();
        while (initTimer.seconds() < 10.0) {
            loaderservo.setPosition(0.0);
        }

        // --- 其他舵机初始位置 ---
        lift.setPosition(0.5);
        center.setPosition(0.5);
    }
}