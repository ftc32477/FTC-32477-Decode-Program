package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

// 🚀 核心引入：引入 Pedro Pathing 的跟随器核心
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * 32477Origin 统一硬件调用滤层类
 * 自动与手动程序均通过本类作为唯一硬件接口，底盘与Odo全权委派给 Constants 统一构建与驱动
 */
public class RobotHardwareV3 {

    // ========== 1. 硬件对象声明（完全保留原有内容，未做任何删改） ==========
    // 底盘动力轮
    public DcMotorEx lf = null;
    public DcMotorEx rf = null;
    public DcMotorEx lb = null;
    public DcMotorEx rb = null;

    // 传感器及里程计
    public IMU hubImu = null;
    public GoBildaPinpointDriver ppointOdo = null;

    // 🚀 【新增滤层核心】：托管全局唯一的 Follower 实例
    public Follower follower = null;

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

    // ========== 2. 核心物理常数 ==========
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(100, 0, 0, 13.5);

    private HardwareMap hwMap = null;

    public RobotHardwareV3() {}

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // =====================================================================
        // 🛠️ 核心滤层实现：底盘与Odo完全通过 Constants 统一获取与构建
        // =====================================================================
        // 1. 让 Constants 底层工厂全权接管硬件映射、正反转配置、零动力刹车及Odo高频闭环
        this.follower = Constants.createFollower(hwMap);

        // 2. 为原有的底盘电机和 Odo 变量指针赋值（指向同一个单例内存，确保原有外部代码不崩）
        this.lf = hwMap.get(DcMotorEx.class, "lf");
        this.rf = hwMap.get(DcMotorEx.class, "rf");
        this.lb = hwMap.get(DcMotorEx.class, "lb");
        this.rb = hwMap.get(DcMotorEx.class, "rb");
        this.ppointOdo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        // 🛑 滤层严禁在此处执行 lf.setDirection() 或 ppointOdo.initialize()！
        // 从而百分之百保护 Constants 里已经调试完毕、注入到 follower 里的所有底层物理配置。

        // 备份保留常态备用 Hub IMU
        hubImu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        hubImu.initialize(parameters);

        // ========== 3. 上层机构硬件初始化（完全保留原有内容，纯净无删改） ==========
        s1 = hwMap.get(DcMotorEx.class, "s1");
        s2 = hwMap.get(DcMotorEx.class, "s2");
        intake = hwMap.get(DcMotor.class, "intake");
        load = hwMap.get(DcMotor.class, "load");

        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        load.setDirection(DcMotor.Direction.FORWARD);

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

        aservo1 = hwMap.get(Servo.class, "aservo1");
        aservo2 = hwMap.get(Servo.class, "aservo2");
        iservo1 = hwMap.get(Servo.class, "iservo1");
        iservo2 = hwMap.get(Servo.class, "iservo2");

        aservo1.setDirection(Servo.Direction.FORWARD);
        aservo2.setDirection(Servo.Direction.REVERSE);
        iservo1.setDirection(Servo.Direction.FORWARD);
        iservo2.setDirection(Servo.Direction.FORWARD);

        aservo1.setPosition(0.4);
        aservo2.setPosition(0.4);
        iservo1.setPosition(0.5);
        iservo2.setPosition(0.5);
    }

    /**
     * 🚀 【新增滤层独占工具】：专供手动遥控程序（TeleOp）调用的底盘驱动滤层函数
     * 让手动程序直接调用 Pedro Pathing 调试好的高阶驱动器，享用完美的麦轮逆运动学及Odo防漂移特性
     *
     * @param strafe  X轴左右平移功率 (对应手柄左摇杆X: gamepad1.left_stick_x)
     * @param forward Y轴前后推行功率 (对应手柄左摇杆Y反转: -gamepad1.left_stick_y)
     * @param turn    旋转转向功率     (对应手柄右摇杆X反转: -gamepad1.right_stick_x)
     * @param fieldCentric 是否开启场体系（开启后无论车头朝哪，摇杆向前车就向场地正前方走！）
     */
    //  正确的修复代码
    public void driveRobot(double strafe, double forward, double turn, boolean fieldCentric) {
        // 将方法名改为 setTeleOpDrive
        // 注意：Pedro Pathing 最后一个参数是 robotCentric（有头模式）
        // 如果主程序传入的是 isFieldCentric（无头模式），这里需要取反（!isFieldCentric）
        follower.setTeleOpDrive(forward, strafe, turn, !fieldCentric);
    }
}