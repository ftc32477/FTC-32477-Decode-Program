package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 战车手动最终版 - 核心控制基类（高优先级空挡硬隔离版）
 * * 核心操控流设计说明：
 * 1. 顶级安全空挡：上电后机构默认完全冻结。只有按下手柄 Back 键后，上层业务逻辑（LB切模式、RT开火、揉球）才会解冻。
 * 2. 左右手操作平衡：左手 LB 一键循环切换大模式（吸球/发射）；右手 RT 在发射模式下作为开火总开（物理开门+失速拦截喂弹）。
 * 3. 角度自瞄校对：长按手柄 A 或 B 键，底盘自动切断手动旋转，利用 Pinpoint 闭环旋转至由子类（红/蓝）指定的场上目标角度。
 */
public class TeleOp_V3_Final_Base extends LinearOpMode {

    protected RobotHardwareV3 robot = new RobotHardwareV3();
    protected IntakeController intakeManager;
    protected ShooterController shooterManager;

    private int currentGear = 1;
    private double iCurrentPosition = 0.5;
    private final double STICK_DEADZONE = 0.08;

    private int driveMode = 0;
    private boolean lastLBState = false;

    // 角度校对 P 控制器常数
    private final double AUTO_AIM_KP = 0.035;
    private final double AUTO_AIM_MAX_TURN = 0.6;

    // 顶级安全总闸：必须按下 Back 键触发为 true 后，机械结构才会脱离空挡工作
    private boolean systemActivated = false;

    // 由红蓝方子类具体赋予的指定校对目标角度
    protected double targetAngleA = 0.0;
    protected double targetAngleB = 90.0;

    @Override
    public void runOpMode() {
        // 1. 初始化统一底层硬件映射
        robot.init(hardwareMap);

        // 2. 双独立管理类并行实例化
        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        waitForStart();

        while (opModeIsActive()) {
            // 每帧自动更新里程计物理坐标
            if (robot.ppointOdo != null) {
                robot.ppointOdo.update();
            }

            // ===================================================================
            // ==========         1. 顶级硬隔离安全监控：Back 激活判定          ==========
            // ===================================================================
            if (gamepad1.back) {
                systemActivated = true; // 一次性敲击，彻底唤醒战车机构
            }

            // ===================================================================
            // ==========         2. 获取实时多维航向角（持续监测）             ==========
            // ===================================================================
            double rawHubYaw = robot.hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rawOdoYaw = 0.0; double odoX = 0.0; double odoY = 0.0;
            if (robot.ppointOdo != null) {
                rawOdoYaw = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }

            // ===================================================================
            // ==========        3. 底盘摇杆控制与 A/B 键自瞄旋转对齐          ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            String targetLogStatus = "MANUAL TURN";
            if (gamepad1.a) {
                // 长按 A 键：闭环锁死目标角度 A
                double error = normalizeAngle(targetAngleA - rawOdoYaw);
                turn = error * AUTO_AIM_KP;
                turn = com.qualcomm.robotcore.util.Range.clip(turn, -AUTO_AIM_MAX_TURN, AUTO_AIM_MAX_TURN);
                targetLogStatus = "🔒 AUTO-AIM ANGLE A [" + targetAngleA + "°]";
            } else if (gamepad1.b) {
                // 长按 B 键：闭环锁死目标角度 B
                double error = normalizeAngle(targetAngleB - rawOdoYaw);
                turn = error * AUTO_AIM_KP;
                turn = com.qualcomm.robotcore.util.Range.clip(turn, -AUTO_AIM_MAX_TURN, AUTO_AIM_MAX_TURN);
                targetLogStatus = "🔒 AUTO-AIM ANGLE B [" + targetAngleB + "°]";
            } else {
                turn = Math.signum(turn) * (turn * turn);
            }

            // 麦轮标准动力学解算
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            double maxChassisPower = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );
            if (maxChassisPower > 1.0) {
                lfPower /= maxChassisPower; rfPower /= maxChassisPower;
                lbPower /= maxChassisPower; rbPower /= maxChassisPower;
            }

            robot.lf.setPower(lfPower); robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower); robot.rb.setPower(rbPower);

            // ===================================================================
            // ==========     ⚠️ 4. 顶级隔离拦截：不按 Back 直接切断控制链     ==========
            // ===================================================================
            if (!systemActivated) {
                // 强制关闭全车除底盘外的所有机构电机，确保空挡绝对静止
                robot.intake.setPower(0.0);
                robot.load.setPower(0.0);
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);

                // 物理大门关闭
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);

                // 发送空挡挂起看板，跳过后续所有大模式切换和按键业务逻辑
                drawTelemetry(false, "NEUTRAL (空挡挂起 - 机构完全冻结)", "WAITING ACTIVATION", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
                continue;
            }

            // ===================================================================
            // ==========       5. 正常业务控制链（按下 Back 解冻后执行）        ==========
            // ===================================================================

            // 【左手：大模式切换机制】
            boolean currentLBState = gamepad1.left_bumper;
            if (currentLBState && !lastLBState) {
                driveMode = (driveMode == 0) ? 1 : 0;
            }
            lastLBState = currentLBState;

            // 【十字键：挡位切换与俯仰角物理联动】
            if (gamepad1.dpad_down) {
                currentGear = 1; iCurrentPosition = 0.10;
            } else if (gamepad1.dpad_left) {
                currentGear = 2; iCurrentPosition = 0.50;
            } else if (gamepad1.dpad_right) {
                currentGear = 3; iCurrentPosition = 0.50;
            } else if (gamepad1.dpad_up) {
                currentGear = 4; iCurrentPosition = 1.00;
            }

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // 【右手化黄金联动调度】
            boolean isRTPressed = gamepad1.right_trigger > 0.1;

            if (driveMode == 0) {
                // 大模式一：常态吸球与时钟揉球
                intakeManager.runIntakeMode();
                // 飞轮在吸球期保持安全的怠速
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                // 大模式二：常态发射准备
                intakeManager.stopOrLock(true); // 0.2 锁球功率保护球道
                // 飞轮全速强起旋，右手 RT 扣下作为最终发射开闸总开关
                shooterManager.updateShooter(currentGear, true, isRTPressed);
            }

            // 渲染正常工作看板
            String driveModeStr = (driveMode == 0) ? "📥 INTAKE MODE ACTIVE" : "🚀 SHOOT MODE READY (RT TO FIRE)";
            drawTelemetry(true, driveModeStr, "RUNNING (机构已全面解冻)", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
        }
    }

    /**
     * 统一控制台看板渲染
     */
    private void drawTelemetry(boolean active, String modeStr, String sysLockStr, String targetLogStatus,
                               double rawHubYaw, double rawOdoYaw, double odoX, double odoY) {
        telemetry.addLine("============ 32477 FINAL MODE INTERFACE ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("Gear Position", "Gear %d", currentGear);
        telemetry.addData("Chassis Lock Status", targetLogStatus);
        telemetry.addLine("----------------------------------------------------");

        if (active) {
            telemetry.addData("Intake System Status", intakeManager.intakeStatus);
            telemetry.addData("S1 Target Drive", "%.1f RPM [%s]", shooterManager.getShooter1RPM(), shooterManager.s1CoreDriverStatus);
            telemetry.addData("S2 Target Drive", "%.1f RPM [%s]", shooterManager.getShooter2RPM(), shooterManager.s2CoreDriverStatus);
            telemetry.addData("Shooter Track Action", shooterManager.shooterTrackStatus);
        } else {
            telemetry.addLine("🚨 警告: 战车机构目前处于安全空挡硬锁状态！");
            telemetry.addLine("🚨 请单次敲击【Back】键解冻上层机械子系统！");
        }
        telemetry.addLine("----------------------------------------------------");

        telemetry.addLine("[🤖 DOUBLE IMU & POSITION VECTOR]");
        telemetry.addData(" -> Hub IMU Yaw", "%.2f °", rawHubYaw);
        telemetry.addData(" -> Pinpoint Heading", "%.2f °", rawOdoYaw);
        telemetry.addData(" -> Local Position", "X: %.1f cm | Y: %.1f cm", odoX, odoY);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}