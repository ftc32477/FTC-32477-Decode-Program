package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TeleOp_V3_Final_Base extends LinearOpMode {

    // 🌟 底层硬件：继续调用你原本未做修改的 RobotHardwareV3
    protected RobotHardwareV3 robot = new RobotHardwareV3();
    protected IntakeController intakeManager;
    protected ShooterController shooterManager;

    private int currentGear = 1;
    private double iCurrentPosition = 0.5;
    private final double STICK_DEADZONE = 0.08;

    private int driveMode = 0;
    private boolean lastLBState = false;
    private boolean systemActivated = false;

    // 🌟 由红蓝子程序直接写入的场地绝对目标角度
    protected double targetAngleA = 0.0;
    protected double targetAngleB = 0.0;

    @Override
    public void runOpMode() {
        // 1. 初始化底盘与管理类
        robot.init(hardwareMap);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        waitForStart();

        while (opModeIsActive()) {
            // 🌟 核心：每轮循环刷新 Pinpoint 传感器的寄存器数据
            if (robot.ppointOdo != null) {
                robot.ppointOdo.update();
            }

            // ===================================================================
            // ==========      🌟 彻底扔掉 Hub IMU，直接采用 Pinpoint 角度       ==========
            // ===================================================================
            double odoHeading = 0.0;
            double odoX = 0.0;
            double odoY = 0.0;

            if (robot.ppointOdo != null) {
                // Pinpoint 弧度转角度，作为全车唯一的绝对航向反馈靶心
                odoHeading = Math.toDegrees(robot.ppointOdo.getHeading());
                odoX = robot.ppointOdo.getPosition().getX(DistanceUnit.CM);
                odoY = robot.ppointOdo.getPosition().getY(DistanceUnit.CM);
            }

            // ===================================================================
            // ==========         1. 顶级硬隔离安全监控：Back 激活判定          ==========
            // ===================================================================
            if (gamepad1.back) {
                systemActivated = true;
            }

            // ===================================================================
            // ==========    2. 底盘麦轮控制与高精度 $1^\circ$ 死区纯 P 自瞄     ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(turn) < STICK_DEADZONE) turn = 0;

            // 手动推力平方非线性 Expo 曲线
            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);

            String targetLogStatus = "MANUAL TURN";

            if (gamepad1.a || gamepad1.b) {
                double targetLoc = gamepad1.a ? targetAngleA : targetAngleB;

                // 🌟 核心修正：符号调转修正正反馈（甩头），直接拿 Pinpoint 角度与目标算差值
                double angleError = normalizeAngle(odoHeading - targetLoc);
                double absError = Math.abs(angleError);

                // 🌟 高精度像素级锁定：缩短至 ±1.0 度绝对死区，进入后彻底清零，强行断电抱死！
                if (absError < 1.0) {
                    turn = 0.0;
                    targetLogStatus = String.format("🔒 LOCK [🎯 TARGET: %.1f°]", targetLoc);
                } else {
                    // 🌟 回归测试通过的第一版经典纯 P 比例控制
                    double kp = 0.01;           // 比例系数，上车可根据底盘摩擦阻力做微调
                    double MAX_TURN_POWER = 0.6; // 最大自瞄转向功率限幅保护

                    double pPower = angleError * kp;

                    // 限幅输出给底盘转向
                    turn = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, pPower));
                    targetLogStatus = String.format("📉 P-ALIGN [🎯 TARGET: %.1f° | ERR: %.1f°]", targetLoc, angleError);
                }
            } else {
                // 松开自瞄，恢复手动操作手完全控制
                turn = Math.signum(turn) * (turn * turn);
            }

            // 麦轮动力复合解算
            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            // 等比例功率限幅
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
            // ==========     ⚠️ 3. 未按 Back 挂起挂空挡                        ==========
            // ===================================================================
            if (!systemActivated) {
                robot.intake.setPower(0.0);
                robot.load.setPower(0.0);
                robot.s1.setPower(0.0);
                robot.s2.setPower(0.0);

                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);

                drawTelemetry(false, "NEUTRAL (空挡挂起)", "WAITING ACTIVATION", targetLogStatus, odoHeading, odoX, odoY);
                continue;
            }

            // ===================================================================
            // ==========       4. 正常业务控制链（双模分发）                   ==========
            // ===================================================================
            boolean currentLBState = gamepad1.left_bumper;
            if (currentLBState && !lastLBState) {
                driveMode = (driveMode == 0) ? 1 : 0; // 快速切换吸取/发射模式
            }
            lastLBState = currentLBState;

            // 挡位控制
            if (gamepad1.dpad_down) { currentGear = 1; iCurrentPosition = 0.10; }
            else if (gamepad1.dpad_left) { currentGear = 2; iCurrentPosition = 0.50; }
            else if (gamepad1.dpad_right) { currentGear = 3; iCurrentPosition = 0.50; }
            else if (gamepad1.dpad_up) { currentGear = 4; iCurrentPosition = 1.00; }

            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            boolean isRTPressed = gamepad1.right_trigger > 0.1;

            if (driveMode == 0) {
                intakeManager.runIntakeMode();
                shooterManager.updateShooter(currentGear, false, false);
            } else {
                intakeManager.stopOrLock(true);
                shooterManager.updateShooter(currentGear, true, isRTPressed);
            }

            String driveModeStr = (driveMode == 0) ? "📥 INTAKE ACTIVE" : "🚀 SHOOT READY";
            drawTelemetry(true, driveModeStr, "RUNNING (正常解冻)", targetLogStatus, odoHeading, odoX, odoY);
        }
    }

    private void drawTelemetry(boolean active, String modeStr, String sysLockStr, String targetLogStatus,
                               double odoHeading, double odoX, double odoY) {
        telemetry.addLine("============ 32477 HIGH PRECISION P-ALIGN Base ============");
        telemetry.addData("★ SYSTEM ACCESS", sysLockStr);
        telemetry.addData("★ DRIVING STATE", modeStr);
        telemetry.addData("Chassis Lock Status", targetLogStatus);
        telemetry.addLine("----------------------------------------------------");

        if (active) {
            telemetry.addData("Intake Status", intakeManager.intakeStatus);
            telemetry.addData("S1 RPM", "%.1f", shooterManager.getShooter1RPM());
            telemetry.addData("S2 RPM", "%.1f", shooterManager.getShooter2RPM());
        }
        telemetry.addLine("----------------------------------------------------");
        telemetry.addLine("[🤖 PPOINT ODO SINGLE SOURCE]");
        telemetry.addData(" -> Pinpoint Heading (当前角)", "%.2f °", odoHeading);
        telemetry.addData(" -> Pinpoint Local Pos", "X: %.1f cm | Y: %.1f cm", odoX, odoY);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}