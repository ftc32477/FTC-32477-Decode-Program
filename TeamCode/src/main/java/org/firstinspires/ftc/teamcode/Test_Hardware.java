package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 更新后的完整硬件类：Test_Hardware
 * 已同步编码器位于后梁中间的物理偏移量算法
 */
public class Test_Hardware {
    public DcMotorEx lf, rf, lb, rb;
    public DcMotorEx intake, load;
    public DcMotorEx s1, s2;
    public IMU imu;
    public GoBildaPinpointDriver odo;

    public int odoResetCount = 0;

    public void init(HardwareMap hwMap) {
        // --- 1. 硬件映射 ---
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");
        intake = hwMap.get(DcMotorEx.class, "intake");
        load = hwMap.get(DcMotorEx.class, "load");
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");

        // --- 2. 底盘电机方向（保留你调试好的侧向安装映射逻辑） ---
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // --- 3. 机构电机方向（保留你最新的反馈逻辑） ---
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);

        // 设置底盘刹车模式
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 4. IMU 初始化（保留你指定的安装方向） ---
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // --- 5. Pinpoint 里程计配置（对接偏移量算法） ---
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        // 设置编码器分辨率 (goBILDA 4-Bar Pod)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /* * 更新算法偏移量 (单位: mm)
         * 物理位置：编码器轮装在行进方向的后梁中间
         * X 偏移量：左右偏移为 0
         * Y 偏移量：车长 32cm，中心到后梁距离为 16cm (160mm)，由于在后方，取 -160.0
         */
        odo.setOffsets(0.0, -160.0);

        // 设置编码器方向
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // 执行硬件状态校验逻辑
        validateOdoHardware();
    }

    /**
     * 校验里程计硬件稳定性，确保初始化时数据归零
     */
    private void validateOdoHardware() {
        boolean isStable = false;
        ElapsedTime timer = new ElapsedTime();
        while (!isStable) {
            odo.resetPosAndIMU();
            odoResetCount++;
            timer.reset();
            isStable = true;
            while (timer.milliseconds() < 800) {
                odo.update();
                Pose2D currentPos = odo.getPosition();
                // 检查 X, Y 在静止状态下是否稳定在 0.1cm 误差范围内
                if (Math.abs(currentPos.getX(DistanceUnit.CM)) > 0.1 ||
                        Math.abs(currentPos.getY(DistanceUnit.CM)) > 0.1) {
                    isStable = false;
                    break;
                }
            }
        }
    }
}