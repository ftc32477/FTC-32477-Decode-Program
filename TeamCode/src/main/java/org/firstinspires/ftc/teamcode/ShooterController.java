package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Shooter 核心控制类 - 智能挡位融合与喂弹回馈版
 * 专注于飞轮闭环轮速、物理大门开闸、物理俯仰角绑定，以及【新增】高精度喂弹状态回馈函数
 */
public class ShooterController {

    private RobotHardwareV3 robot;
    private ElapsedTime servoOpenTimer = new ElapsedTime(); // 用于舵机完全打开的0.5秒延时计时
    private boolean wasRTPressedLastFrame = false;          // 记录上一帧RT状态，用于捕捉边缘触发

    // 发射子状态机定义
    private enum FiringState {
        READY_TO_START,     // 等待触发/初始状态
        WAITING_FOR_SERVO,  // 速度达标，已开门，正在等待0.5秒舵机完全打开
        SHOOTING            // 门已完全打开，全力喂弹中
    }
    private FiringState currentFiringState = FiringState.READY_TO_START;

    // ===================================================================
    // ==========          1. 固化实车调测完成的 PIDF 阵列          ==========
    // ===================================================================
    private final PIDFCoefficients PIDF_IDLE_COMMON = new PIDFCoefficients(13.2, 0, 0.5, 17.0);
    private final PIDFCoefficients PIDF_GEAR_1 = new PIDFCoefficients(13.2, 0.0, 0.1, 23.7);
    private final PIDFCoefficients PIDF_GEAR_2 = new PIDFCoefficients(13.0, 0.0, 0.2, 20.0);
    private final PIDFCoefficients PIDF_GEAR_3 = new PIDFCoefficients(18.1, 0.0, 0.1, 22.6);
    private final PIDFCoefficients PIDF_GEAR_4 = new PIDFCoefficients(18.5, 0.0, 0.5, 22.0);

    // ===================================================================
    // ==========            2. 目标转速与俯仰角度参数阵列            ==========
    // ===================================================================
    private final double GEAR_1_TARGET = 1550.0; private final double GEAR_1_IDLE = 1300.0;
    private final double GEAR_2_TARGET = 1700.0; private final double GEAR_2_IDLE = 1500.0;
    private final double GEAR_3_TARGET = 2000.0; private final double GEAR_3_IDLE = 1700.0;
    private final double GEAR_4_TARGET = 2100.0; private final double GEAR_4_IDLE = 1800.0;

    private final double GEAR_1_ANGLE = 0.10;
    private final double GEAR_2_ANGLE = 0.50;
    private final double GEAR_3_ANGLE = 0.50;
    private final double GEAR_4_ANGLE = 1.00;

    private final double BANGBANG_TRIGGER_THRESHOLD = 50.0;
    private final double RPM_TOLERANCE_LOWER = 50.0;
    private final double RPM_TOLERANCE_UPPER = 150.0;
    private final double SERVO_OPEN_DELAY_SEC = 0.5; // 舵机完全打开所需的延时时间（秒）

    private boolean isFirstAcceleration = true;
    private PIDFCoefficients activePIDF = null;

    public String s1CoreDriverStatus = "STANDBY";
    public String s2CoreDriverStatus = "STANDBY";
    public String shooterTrackStatus = "IDLE";

    // 内部控制链状态旗标
    private boolean interceptingFlag = false;
    private boolean feedingActiveFlag = false; // 【新增】喂弹激活状态标志位

    public ShooterController(RobotHardwareV3 hardware) {
        this.robot = hardware;
    }

    /**
     * ✨【新增】重载构造函数：专门无缝兼容自动程序硬件类
     */
    public ShooterController(RobotHardwareV3_Auto hardware) {
        // 1. 创建干净的 V3 外壳，它会天然继承父类声明的 SHOOTER_TICKS_PER_REV = 28.0 物理常数
        this.robot = new RobotHardwareV3();

        // 2. 批量将自动硬件类中初始化完毕的飞轮、舵机及弹道马达指针转移进外壳
        this.robot.s1 = hardware.s1;
        this.robot.s2 = hardware.s2;
        this.robot.aservo1 = hardware.aservo1;
        this.robot.aservo2 = hardware.aservo2;
        this.robot.iservo1 = hardware.iservo1;
        this.robot.iservo2 = hardware.iservo2;
        this.robot.intake = hardware.intake;
        this.robot.load = hardware.load;
    }

    /**
     * 智能复合发射闭环与推弹状态机
     */
    public void updateShooter(int gear, boolean requestSpinUp, boolean requestFire) {
        if (robot == null || robot.s1 == null || robot.s2 == null) return;

        // --- 1. 挡位映射 ---
        double targetRPM; double currentGearIdle; PIDFCoefficients gearActivePIDF; double targetAngle;
        switch (gear) {
            case 2:
                targetRPM = GEAR_2_TARGET; currentGearIdle = GEAR_2_IDLE; gearActivePIDF = PIDF_GEAR_2; targetAngle = GEAR_2_ANGLE;
                break;
            case 3:
                targetRPM = GEAR_3_TARGET; currentGearIdle = GEAR_3_IDLE; gearActivePIDF = PIDF_GEAR_3; targetAngle = GEAR_3_ANGLE;
                break;
            case 4:
                targetRPM = GEAR_4_TARGET; currentGearIdle = GEAR_4_IDLE; gearActivePIDF = PIDF_GEAR_4; targetAngle = GEAR_4_ANGLE;
                break;
            default:
                targetRPM = GEAR_1_TARGET; currentGearIdle = GEAR_1_IDLE; gearActivePIDF = PIDF_GEAR_1; targetAngle = GEAR_1_ANGLE;
                break;
        }

        if (robot.iservo1 != null && robot.iservo2 != null) {
            robot.iservo1.setPosition(targetAngle);
            robot.iservo2.setPosition(1.0 - targetAngle);
        }

        double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
        double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

        // --- 2. 飞轮闭环内核决策 ---
        double currentLoopTargetRPM; PIDFCoefficients expectedPIDF;
        if (requestSpinUp) {
            currentLoopTargetRPM = targetRPM; expectedPIDF = gearActivePIDF;
            double targetTicksPerSec = (currentLoopTargetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;
            double currentError1 = currentLoopTargetRPM - actualRPM1;
            double currentError2 = currentLoopTargetRPM - actualRPM2;

            if (isFirstAcceleration) {
                if (actualRPM1 >= (targetRPM - 15.0) && actualRPM2 >= (targetRPM - 15.0)) isFirstAcceleration = false;
            }

            if (isFirstAcceleration) {
                robot.s1.setPower(1.0); s1CoreDriverStatus = "🔥 BANG-BANG [初次起旋]";
            } else if (currentError1 >= BANGBANG_TRIGGER_THRESHOLD) {
                robot.s1.setPower(1.0); s1CoreDriverStatus = "⚡ BANG-BANG [失速补偿]";
            } else {
                robot.s1.setVelocity(targetTicksPerSec); s1CoreDriverStatus = "⚙️ PIDF [精准控速]";
            }

            if (isFirstAcceleration) {
                robot.s2.setPower(1.0); s2CoreDriverStatus = "🔥 BANG-BANG [初次起旋]";
            } else if (currentError2 >= BANGBANG_TRIGGER_THRESHOLD) {
                robot.s2.setPower(1.0); s2CoreDriverStatus = "⚡ BANG-BANG [失速补偿]";
            } else {
                robot.s2.setVelocity(targetTicksPerSec); s2CoreDriverStatus = "⚙️ PIDF [精准控速]";
            }
        } else {
            currentLoopTargetRPM = currentGearIdle; expectedPIDF = PIDF_IDLE_COMMON;
            double targetTicksPerSec = (currentLoopTargetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;
            robot.s1.setVelocity(targetTicksPerSec); robot.s2.setVelocity(targetTicksPerSec);
            s1CoreDriverStatus = "💤 PIDF [对应挡位怠速]";
            s2CoreDriverStatus = "💤 PIDF [对应挡位怠速]";
            isFirstAcceleration = true;
        }

        if (activePIDF != expectedPIDF) {
            robot.s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, expectedPIDF);
            robot.s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, expectedPIDF);
            activePIDF = expectedPIDF;
        }

        // --- 3. 实时速度达标检测 ---
        boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean isSpeedNowReady = s1SpeedReady && s2SpeedReady;

        // --- 4. 边缘捕捉：当松开发射请求时，强制重置状态机 ---
        if (!requestFire) {
            currentFiringState = FiringState.READY_TO_START;
            feedingActiveFlag = false; // 只要发射清零，回馈值立即恢复为 false
        }

        // ===================================================================
        // ==========          5. 核心：时序复合控制状态机          ==========
        // ===================================================================
        if (requestFire) {
            robot.aservo1.setPosition(0.0);
            robot.aservo2.setPosition(0.0);

            switch (currentFiringState) {
                case READY_TO_START:
                    feedingActiveFlag = false; // 初始/未到速时为 false
                    if (isSpeedNowReady) {
                        robot.intake.setPower(0.0);
                        robot.load.setPower(0.0);
                        servoOpenTimer.reset();
                        currentFiringState = FiringState.WAITING_FOR_SERVO;
                        shooterTrackStatus = "⏳ OPENING [转速达标：开门静待0.5s]";
                        interceptingFlag = false;
                    } else {
                        robot.intake.setPower(0.0);
                        robot.load.setPower(0.0);
                        shooterTrackStatus = "⚠️ WAIT_RPM [转速未达标：初次拦截]";
                        interceptingFlag = true;
                    }
                    break;

                case WAITING_FOR_SERVO:
                    feedingActiveFlag = false; // 舵机门在打开的 0.5s 计时期间，依旧保持为 false
                    robot.intake.setPower(0.0);
                    robot.load.setPower(0.0);
                    shooterTrackStatus = "⏳ OPENING [开门计时中: " + String.format("%.2f", servoOpenTimer.seconds()) + "s]";
                    interceptingFlag = false;

                    if (servoOpenTimer.seconds() >= SERVO_OPEN_DELAY_SEC) {
                        currentFiringState = FiringState.SHOOTING;
                    }
                    break;

                case SHOOTING:
                    if (!isSpeedNowReady) {
                        robot.intake.setPower(0.0);
                        robot.load.setPower(0.0);
                        shooterTrackStatus = "⚠️ INTERCEPT [发射中掉速：球道拦截中]";
                        interceptingFlag = true;
                        // 可选设计：掉速拦截时是否挂起回馈？由于此时球道停止工作，建议同步置为 false，回速后再为 true
                        feedingActiveFlag = false;
                    } else {
                        robot.intake.setPower(0.9);
                        robot.load.setPower(0.9);
                        shooterTrackStatus = "🚀 FIRE [完全开启：持续推弹中]";
                        interceptingFlag = false;

                        // ✨【核心实现】门开满0.5秒且转速达标，球道正常工作，回馈值正式变为 true
                        feedingActiveFlag = true;
                    }
                    break;
            }
        } else {
            // --- 6. 释放复位阶段 ---
            robot.aservo1.setPosition(0.4);
            robot.aservo2.setPosition(0.4);
            shooterTrackStatus = "WAITING FIRE TRIGGER";
            interceptingFlag = false;
            feedingActiveFlag = false; // 兜底复位
        }

        wasRTPressedLastFrame = requestFire;
    }

    // ===================================================================
    // ==========          6. 提供给主程序的回复接口          ==========
    // ===================================================================

    /**
     * 【新功能】到速与喂弹就绪提示回馈
     * 当且仅当大门开闸满0.5秒、转速达标且球道开始推弹瞬间变为 true，松开按键后自动恢复为 false
     * @return boolean 是否正在全力喂弹
     */
    public boolean isFeeding() {
        return feedingActiveFlag;
    }

    public boolean isIntercepting() { return interceptingFlag; }

    public double getShootHoldTime() {
        if (currentFiringState == FiringState.WAITING_FOR_SERVO) {
            return servoOpenTimer.seconds();
        } else if (currentFiringState == FiringState.SHOOTING) {
            return SERVO_OPEN_DELAY_SEC;
        }
        return 0.0;
    }

    public String getShooterStatusStr() { return shooterTrackStatus; }
    public double getShooter1RPM() { return (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
    public double getShooter2RPM() { return (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
}