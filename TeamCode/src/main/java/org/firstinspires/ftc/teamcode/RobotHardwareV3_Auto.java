package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 32477Origin 专属自动机构硬件类 - 剥离底盘与Odo版
 * 规避双重初始化冲突，底盘与Odo全权交给 Pedro Pathing 的 Constants 管理
 */
public class RobotHardwareV3_Auto {

    // ========== 1. 仅保留上层机构硬件对象声明 ==========
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

    // ========== 2. 核心物理常数（严格保留原有内容） ==========
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(100, 0, 0, 13.5);

    private HardwareMap hwMap = null;

    public RobotHardwareV3_Auto() {}

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // ========== 3. 仅初始化上层机构电机与舵机（完全复制自原版，绝无删改） ==========
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
}