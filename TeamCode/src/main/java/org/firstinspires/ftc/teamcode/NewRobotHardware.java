package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class NewRobotHardware {
    // 1. 声明硬件对象
    public DcMotorEx lf, rf, lb, rb;
    public IMU imu;

    // 子系统电机与舵机
    public DcMotor intake;       // 拾取电机
    public DcMotor load;         // 装填电机
    public DcMotorEx s1, s2;     // 双发射电机
    public Servo pitchServo;     // 发射俯仰角舵机

    // 新增：GoBilda Pinpoint 里程计计算机
    public GoBildaPinpointDriver odoComputer;

    public void init(HardwareMap hwMap) {
        // --- A. 硬件映射 (Hardware Mapping) ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");

        intake = hwMap.get(DcMotor.class, "intake");
        load   = hwMap.get(DcMotor.class, "load");
        s1     = hwMap.get(DcMotorEx.class, "s1");
        s2     = hwMap.get(DcMotorEx.class, "s2");
        pitchServo = hwMap.get(Servo.class, "pitchServo");

        imu = hwMap.get(IMU.class, "imu");
        odoComputer = hwMap.get(GoBildaPinpointDriver.class, "odocomputer");

        // --- B. 底盘电机配置 ---
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- C. 子系统电机配置 ---
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);
        s1.setDirection(DcMotor.Direction.FORWARD);
        s2.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 发射电机通常使用 Encoder 闭环控制
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- D. 传感器初始化 ---
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        odoComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // 注意：Offset 需根据实际物理位置调整
        odoComputer.setOffsets(0.0, 0.0);
        odoComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odoComputer.resetPosAndIMU();
    }
}