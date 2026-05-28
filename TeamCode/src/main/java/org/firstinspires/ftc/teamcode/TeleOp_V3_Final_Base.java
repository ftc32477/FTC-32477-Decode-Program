package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 32477 战车手动最终版 - 核心控制基类（极致解耦·继承自动级PIDF自瞄版）
 * * 物理特性：
 * 完美的自瞄拉力与高阻尼刹车感完全来源于你们自动档的 Constants 调校参数，彻底斩断对外部 Pedro Pathing 库路径的依赖。
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

    // ===================================================================
    // ========== 🎯 直接提取自自动 Constants.java 的核心底盘 PIDF 阵列 ==========
    // ===================================================================
    // 完美的刚度与物理微分阻尼，用来克服静摩擦并直接刹退左右疯狂甩头
    private final double AUTO_AIM_KP = 0.032;   // 继承自 headingPIDFCoefficients 的 P 项
    private final double AUTO_AIM_KD = 0.0018;  // 继承自 headingPIDFCoefficients 的 D 项
    private final double AUTO_AIM_MAX_TURN = 0.65; // 最大偏航角物理功率限制

    private double lastAngleError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime(); // 高精度差分时钟，用以消除积分和微分抖动

    // 顶级安全总闸：必须按下 Back 键触发为 true 后，机械结构才会脱离空挡工作
    private boolean systemActivated = false;

    // 由红蓝方子类具体赋予的指定校对目标角度（回归传统的标准度数制 0~360°）
    protected double targetAngleA = 0.0;
    protected double targetAngleB = 90.0;

    @Override
    public void runOpMode() {
        // 1. 初始化统一底层硬件映射
        robot.init(hardwareMap);

        // 2. 双独立管理类并行实例化（ShooterController保持纯净未加锁隔离的分离版）
        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        pidTimer.reset();
        waitForStart();

        while (opModeIsActive()) {
            // 每帧自动更新里程计物理坐标
            if (robot.ppointOdo != null) {
                robot.ppointOdo.update();
            }

            // 计算精准的时钟步长，彻底喂饱 D 项微分阻尼器
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt <= 0) dt = 0.005;

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
            // ==========        3. 底盘麦轮遥感控制与高级 PD 自瞄闭环          ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            // 标准推力二次 Expo 曲线映射
            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            String targetLogStatus = "MANUAL TURN";

            if (gamepad1.a || gamepad1.b) {
                // 自动提取对应的红蓝方目标角度（度数制）
                double targetLoc = gamepad1.a ? targetAngleA : targetAngleB;
                double currentError = normalizeAngle(targetLoc - rawOdoYaw);

                // 🌟 高级算法前馈：误差变率计算。逼近目标时 errorDerivative 会变负，产生天然的液压刹车阻尼
                double errorDerivative = (currentError - lastAngleError) / dt;

                // 经典自瞄控制方程：拉力项 + 阻尼项。彻底消除在一定区间内疯狂甩头不减弱的等幅震荡
                turn = (currentError * AUTO_AIM_KP) + (errorDerivative * AUTO_AIM_KD);
                turn = com.qualcomm.robotcore.util.Range.clip(turn, -AUTO_AIM_MAX_TURN, AUTO_AIM_MAX_TURN);

                lastAngleError = currentError;
                targetLogStatus = gamepad1.a ? "🔒 EXPERT LOCK ANGLE A" : "🔒 EXPERT LOCK ANGLE B";
            } else {
                // 无自瞄时恢复纯手动旋转
                turn = Math.signum(turn) * (turn * turn);
                lastAngleError = 0.0; // 离手重置历史误差，防止再次按下时引发剧烈突变
            }

            // 麦克纳姆轮经典运动学动力分解
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
                // 强制锁定关闭全车除底盘外的所有机构电机，确保空挡绝对安全静止
                robot.intake.setPower(0.0);
                robot.load.setPower(0.0);
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);

                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);

                drawTelemetry(false, "NEUTRAL (空挡挂起 - 机械全解耦锁死)", "WAITING ACTIVATION", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
                continue;
            }

            // ===================================================================
            // ==========       5. 正常业务控制链（按下 Back 解冻后执行）        ==========
            // ===================================================================

            // 【左手 LB 大模式高效一键切换】
            boolean currentLBState = gamepad1.left_bumper;
            if (currentLBState && !lastLBState) {
                driveMode = (driveMode == 0) ? 1 : 0;
            }
            lastLBState = currentLBState;

            // 【十字键挡位快速控制与俯仰倾角物理联动】
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

            // 【右手化黄金联动调度与失速拦截执行】
            boolean isRTPressed = gamepad1.right_trigger > 0.1;

            if (driveMode == 0) {
                // 模式一：吸球模式（时钟吸揉机制启动）
                intakeManager.runIntakeMode();
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                // 模式二：发射准备模式
                intakeManager.stopOrLock(true); // 0.2 微功率防滑锁弹
                shooterManager.updateShooter(currentGear, true, isRTPressed);
            }

            String driveModeStr = (driveMode == 0) ? "📥 INTAKE MODE ACTIVE" : "🚀 SHOOT MODE READY";
            drawTelemetry(true, driveModeStr, "RUNNING (子系统完全解冻)", targetLogStatus, rawHubYaw, rawOdoYaw, odoX, odoY);
        }
    }

    private void drawTelemetry(boolean active, String modeStr, String sysLockStr, String targetLogStatus,
                               double rawHubYaw, double rawOdoYaw, double odoX, double odoY) {
        telemetry.addLine("============ 32477 EXPERT MODE INTERFACE ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("Chassis Lock Status", targetLogStatus);
        telemetry.addLine("----------------------------------------------------");

        if (active) {
            telemetry.addData("Intake System Status", intakeManager.intakeStatus);
            telemetry.addData("S1 Target Drive", "%.1f RPM", shooterManager.getShooter1RPM());
            telemetry.addData("S2 Target Drive", "%.1f RPM", shooterManager.getShooter2RPM());
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