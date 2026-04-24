package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Test_Hardware {
    public DcMotorEx lf, rf, lb, rb;
    public DcMotorEx intake, load;
    public DcMotorEx s1, s2;
    public IMU imu;
    public GoBildaPinpointDriver odo;

    public int odoResetCount = 0;

    public void init(HardwareMap hwMap) {
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lb = hwMap.get(DcMotorEx.class, "lb");
        rb = hwMap.get(DcMotorEx.class, "rb");
        intake = hwMap.get(DcMotorEx.class, "intake");
        load = hwMap.get(DcMotorEx.class, "load");
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");

        // 底盘电机方向（沿用你调试好的侧向安装映射逻辑）
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // --- 机构电机方向更新 ---
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.REVERSE);   // 根据反馈：由 FORWARD 改为 REVERSE
        s1.setDirection(DcMotor.Direction.REVERSE);     // 根据反馈：由 FORWARD 改为 REVERSE
        s2.setDirection(DcMotor.Direction.FORWARD);     // 根据反馈：由 REVERSE 改为 FORWARD

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU 初始化并强制重置
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Pinpoint 初始化
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-120.0, -120.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        validateOdoHardware();
    }

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
                if (Math.abs(currentPos.getX(DistanceUnit.CM)) > 0.1 ||
                        Math.abs(currentPos.getY(DistanceUnit.CM)) > 0.1) {
                    isStable = false;
                    break;
                }
            }
        }
    }
}