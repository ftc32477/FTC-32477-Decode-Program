package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class NewRobotHardware {
    // 1. 声明硬件对象
    public DcMotorEx lf, rf, lb, rb;
    public IMU imu;

    // 新增：GoBilda Pinpoint 里程计计算机
    public GoBildaPinpointDriver odoComputer;

    // 物理常数 (你可以根据新车需求随时添加，比如轮子直径等)
    // 提示：4-Bar 的 ticks-per-mm 常数已经内置在 GoBilda 的驱动库中了，不需要在这里重复计算

    public void init(HardwareMap hwMap) {
        // --- A. 硬件映射 (Hardware Mapping) ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");

        imu = hwMap.get(IMU.class, "imu");

        // 映射 Pinpoint 计算机 (请确保在 Driver Station 上将其命名为 "odocomputer")
        odoComputer = hwMap.get(GoBildaPinpointDriver.class, "odocomputer");

        // --- B. 底盘电机配置 (对齐 Test_Chassis_4_0 测试结果) ---
        // 横置底盘的正确运转方向
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // 统一设置为刹车模式，提升操控精准度
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- C. 传感器初始化 ---

        // 1. Control Hub 内部 IMU 初始化 (保留了旧车的摆放方向配置)
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // 2. GoBilda Pinpoint 初始化
        /* * 设定为 4-Bar 追踪轮。
         * 这会自动调用底层库中 19.89436789 ticks-per-mm 的超高精度参数。
         */
        odoComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * 追踪轮偏移量设置 (X, Y) - 单位：毫米 (mm)
         * 【重要】这里目前填的是 0.0，你需要根据新车的实际机械结构亲自测量并修改它！
         * X 偏移：前向测量轮（记录前进后退）距离机器人旋转中心的横向距离。中心偏左为正，偏右为负。
         * Y 偏移：侧向测量轮（记录左右平移）距离机器人旋转中心的前后距离。中心偏前为正，偏后为负。
         */
        odoComputer.setOffsets(0.0, 0.0);

        // 默认编码器方向。如果在推车测试时发现遥测仪上显示的坐标正负号反了，把对应的改成 REVERSED 即可。
        odoComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // 初始化时，重置 Pinpoint 的内部坐标系和内置陀螺仪零点
        odoComputer.resetPosAndIMU();
    }
}