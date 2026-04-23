package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class NewRobotHardware {
    /* 硬件对象声明 */
    public DcMotorEx lf, rf, lb, rb;   // 底盘
    public DcMotorEx intake, load;    // 吸取与传输
    public DcMotorEx s1, s2;          // 双发射轮
    public IMU imu;                   // Hub 内置 IMU
    public GoBildaPinpointDriver odo; // Pinpoint 计算机

    public void init(HardwareMap hwMap) {
        // 1. 电机映射
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");
        intake = hwMap.get(DcMotorEx.class, "intake");
        load = hwMap.get(DcMotorEx.class, "load");
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");

        // 2. 底盘方向设置 (横置底盘逻辑)
        // 根据测试反馈，我们需要调整方向以适配特定的平移算法
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // 3. 辅助机构方向
        s1.setDirection(DcMotor.Direction.FORWARD);
        s2.setDirection(DcMotor.Direction.REVERSE); // 双轮对转
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.FORWARD);

        // 4. 设置零功率行为 (刹车模式)
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 5. IMU 初始化 (Logo向左, USB向后)
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // 6. Pinpoint (odo) 初始化与自重启
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        // 设置偏移量 (根据基线 V5.0，请在实测后微调这些值)
        odo.setOffsets(0.0, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        validateOdo(); // 执行基线规范要求的自重启校验
    }

    private void validateOdo() {
        // 重置并静置，确保传感器启动时数据干净
        odo.resetPosAndIMU();
        try { Thread.sleep(300); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}