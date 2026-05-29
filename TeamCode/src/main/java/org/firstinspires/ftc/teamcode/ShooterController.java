package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Shooter 核心控制类 - 最终兼容版
 * 专注于飞轮闭轮速、物理大门开闸以及发射推弹瞬间的失速物理拦截
 * * 升级特性：
 * 1. 引入开门0.5秒延时等待机制，确保舵机完全到位后再行喂弹。
 * 2. 优化失速拦截逻辑，掉速时不关门，仅拦截球道，回速后自动恢复。
 * 3. 松开RT或退出时自动复位舵机与状态机。
 * 4. 【已修复】补回所有 TeleOp 所需接口：getShootHoldTime()、isIntercepting() 以及状态变量。
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
    private final PIDFCoefficients PIDF_GEAR_2 = new PIDFCoefficients(13.0, 0.0, 0.2, 22.8);
    private final PIDFCoefficients PIDF_GEAR_3 = new PIDFCoefficients(18.1, 0.0, 0.1, 22.6);
    private final PIDFCoefficients PIDF_GEAR_4 = new PIDFCoefficients(18.5, 0.0, 0.5, 22.0);

    // ===================================================================
    // ==========                2. 目标转速参数阵列                ==========
    // ===================================================================
    private final double GEAR_1_TARGET = 1550.0; private final double GEAR_1_IDLE = 1300.0;
    private final double GEAR_2_TARGET = 1700.0; private final double GEAR_2_IDLE = 1500.0;
    private final double GEAR_3_TARGET = 2000.0; private final double GEAR_3_IDLE = 1700.0;
    private final double GEAR_4_TARGET = 2100.0; private final double GEAR_4_IDLE = 1800.0;

    private final double BANGBANG_TRIGGER_THRESHOLD = 50.0;
    private final double RPM_TOLERANCE_LOWER = 30.0;
    private final double RPM_TOLERANCE_UPPER = 150.0;
    private final double SERVO_OPEN_DELAY_SEC = 0.5; // 舵机完全打开所需的延时时间（秒）

    private boolean isFirstAcceleration = true;
    private PIDFCoefficients activePIDF = null;

    public String s1CoreDriverStatus = "STANDBY";
    public String s2CoreDriverStatus = "STANDBY";
    public String shooterTrackStatus = "IDLE"; // 提供给 TeleOp 调用的公开状态字符串

    // 内部拦截状态旗标
    private boolean interceptingFlag = false;

    public ShooterController(RobotHardwareV3 hardware) {
        this.robot = hardware;
    }

    /**
     * 纯净发射闭环与推弹状态机
     * @param gear              当前分挡 (1-4)
     * @param requestSpinUp     是否请求起旋（发射模式常态置为 true；吸球模式下若要怠速，由TeleOp传入对应逻辑）
     * @param requestFire       是否按下发射键（由右手的 RT 触发控制）
     */
    public void updateShooter(int gear, boolean requestSpinUp, boolean requestFire) {
        if (robot == null || robot.s1 == null || robot.s2 == null) return;

        // --- 挡位映射 ---
        double targetRPM; double currentGearIdle; PIDFCoefficients gearActivePIDF;
        switch (gear) {
            case 2: targetRPM = GEAR_2_TARGET; currentGearIdle = GEAR_2_IDLE; gearActivePIDF = PIDF_GEAR_2; break;
            case 3: targetRPM = GEAR_3_TARGET; currentGearIdle = GEAR_3_IDLE; gearActivePIDF = PIDF_GEAR_3; break;
            case 4: targetRPM = GEAR_4_TARGET; currentGearIdle = GEAR_4_IDLE; gearActivePIDF = PIDF_GEAR_4; break;
            default: targetRPM = GEAR_1_TARGET; currentGearIdle = GEAR_1_IDLE; gearActivePIDF = PIDF_GEAR_1; break;
        }

        double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
        double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

        // --- 飞轮闭环内核决策 ---
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
            // 当 requestSpinUp 为 false 时，按照对应挡位怠速运转
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

        // --- 实时速度达标检测 ---
        boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean isSpeedNowReady = s1SpeedReady && s2SpeedReady;

        // --- 边缘捕捉：当松开RT时，强制重置状态机 ---
        if (!requestFire) {
            currentFiringState = FiringState.READY_TO_START;
        }

        // ===================================================================
        // ==========          3. 核心：时序复合控制状态机          ==========
        // ===================================================================
        if (requestFire) {
            // 只要按下RT，大门必须保持完全开启（无论是否掉速都不自动关闭）
            robot.aservo1.setPosition(0.0);
            robot.aservo2.setPosition(0.0);

            switch (currentFiringState) {
                case READY_TO_START:
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
                        interceptingFlag = true; // 转速未达标触发物理拦截旗标
                    }
                    break;

                case WAITING_FOR_SERVO:
                    robot.intake.setPower(0.0);
                    robot.load.setPower(0.0);
                    shooterTrackStatus = "⏳ OPENING [开门计时中: " + String.format("%.2f", servoOpenTimer.seconds()) + "s]";
                    interceptingFlag = false; // 处于开门等待期间，不算异常失速拦截

                    if (servoOpenTimer.seconds() >= SERVO_OPEN_DELAY_SEC) {
                        currentFiringState = FiringState.SHOOTING;
                    }
                    break;

                case SHOOTING:
                    if (!isSpeedNowReady) {
                        // 💥 动态失速物理拦截
                        robot.intake.setPower(0.0);
                        robot.load.setPower(0.0);
                        shooterTrackStatus = "⚠️ INTERCEPT [发射中掉速：球道拦截中]";
                        interceptingFlag = true; // 发射中遭遇掉速，拦截旗标置为 true
                    } else {
                        robot.intake.setPower(0.9);
                        robot.load.setPower(0.9);
                        shooterTrackStatus = "🚀 FIRE [完全开启：持续推弹中]";
                        interceptingFlag = false; // 速度恢复，正常推弹
                    }
                    break;
            }
        } else {
            // --- 4. 释放复位阶段（松开RT或进入Intake模式） ---
            robot.aservo1.setPosition(0.4); // 大门关闭
            robot.aservo2.setPosition(0.4);
            shooterTrackStatus = "WAITING FIRE TRIGGER";
            interceptingFlag = false; // 松开后不再拦截
        }

        wasRTPressedLastFrame = requestFire; // 记录帧状态
    }

    // ===================================================================
    // ==========          4. 提供给主程序的回复接口          ==========
    // ===================================================================

    /**
     * 【新修复】完美补回提供给主程序的拦截状态反馈函数
     * 当处于发射按下状态、且由于飞轮掉速导致球道停止推弹时，返回 true
     */
    public boolean isIntercepting() {
        return interceptingFlag;
    }

    /**
     * 补回主程序遥测面板所调用的获取开火保持/延时计时函数
     */
    public double getShootHoldTime() {
        if (currentFiringState == FiringState.WAITING_FOR_SERVO) {
            return servoOpenTimer.seconds();
        } else if (currentFiringState == FiringState.SHOOTING) {
            return SERVO_OPEN_DELAY_SEC;
        }
        return 0.0;
    }

    public String getShooterStatusStr() {
        return shooterTrackStatus;
    }

    public double getShooter1RPM() { return (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
    public double getShooter2RPM() { return (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
}