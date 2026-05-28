package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * 32477 Shooter 核心控制类 - 纯净发射解耦版
 * 专注于飞轮闭轮速、物理大门开闸以及发射推弹瞬间的失速物理拦截
 */
public class ShooterController {

    private RobotHardwareV3 robot;

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

    private boolean isFirstAcceleration = true;
    private PIDFCoefficients activePIDF = null;

    public String s1CoreDriverStatus = "STANDBY";
    public String s2CoreDriverStatus = "STANDBY";
    public String shooterTrackStatus = "IDLE";

    public ShooterController(RobotHardwareV3 hardware) {
        this.robot = hardware;
    }

    /**
     * 纯净发射闭环与推弹状态机
     * @param gear              当前分挡 (1-4)
     * @param requestSpinUp     是否请求起旋（发射模式常态置为 true）
     * @param requestFire       是否按下发射键（修改后由右手的 RT 触发控制）
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
            currentLoopTargetRPM = currentGearIdle; expectedPIDF = PIDF_IDLE_COMMON;
            double targetTicksPerSec = (currentLoopTargetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;
            robot.s1.setVelocity(targetTicksPerSec); robot.s2.setVelocity(targetTicksPerSec);
            s1CoreDriverStatus = "💤 PIDF [低能耗怠速]";
            s2CoreDriverStatus = "💤 PIDF [低能耗怠速]";
            isFirstAcceleration = true;
        }

        if (activePIDF != expectedPIDF) {
            robot.s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, expectedPIDF);
            robot.s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, expectedPIDF);
            activePIDF = expectedPIDF;
        }

        // --- 舵机物理大门控制（按下开火键时即完全开启 0.0，无论是否失速都不自动关闭） ---
        if (requestFire) {
            robot.aservo1.setPosition(0.0);
            robot.aservo2.setPosition(0.0);
        } else {
            robot.aservo1.setPosition(0.4);
            robot.aservo2.setPosition(0.4);
        }

        // --- 球道送弹与物理失速拦截控制 ---
        if (requestFire) {
            boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean isSpeedNowReady = s1SpeedReady && s2SpeedReady;

            if (!isSpeedNowReady) {
                // 💥 触发动态失速物理拦截：开门状态下如果转速不达标，球道切断动力，防止卡死
                robot.intake.setPower(0.0);
                robot.load.setPower(0.0);
                shooterTrackStatus = "⚠️ INTERCEPT [转速跌落：拦截球道]";
            } else {
                // 速度达标，全功率喂弹
                robot.intake.setPower(0.9);
                robot.load.setPower(0.9);
                shooterTrackStatus = "🚀 FIRE [速度达标：推弹入膛]";
            }
        } else {
            shooterTrackStatus = "WAITING FIRE TRIGGER";
        }
    }

    public double getShooter1RPM() { return (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
    public double getShooter2RPM() { return (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0; }
}