package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp_V3_newTest", group = "Production")
public class TeleOp_V3_newTest extends LinearOpMode {

    // ===================================================================
    // ==========            一、内嵌硬件类定义 (Hardware)          ==========
    // ===================================================================
    public static class RobotHardware {
        public DcMotorEx lf = null;
        public DcMotorEx rf = null;
        public DcMotorEx lb = null;
        public DcMotorEx rb = null;
        public IMU hubImu = null;

        public DcMotorEx s1 = null;
        public DcMotorEx s2 = null;
        public Servo aservo1 = null;
        public Servo aservo2 = null;
        public Servo iservo1 = null;
        public Servo iservo2 = null;

        public DcMotor intake = null;
        public DcMotor load = null;

        public final double SHOOTER_TICKS_PER_REV = 28.0;
        public final PIDFCoefficients CHASSIS_PIDF = new PIDFCoefficients(12.5, 3.0, 0.5, 11.5);

        public void init(HardwareMap hwMap) {
            lf = hwMap.get(DcMotorEx.class, "lf");
            rf = hwMap.get(DcMotorEx.class, "rf");
            lb = hwMap.get(DcMotorEx.class, "lb");
            rb = hwMap.get(DcMotorEx.class, "rb");

            lf.setDirection(DcMotor.Direction.REVERSE);
            lb.setDirection(DcMotor.Direction.REVERSE);
            rf.setDirection(DcMotor.Direction.FORWARD);
            rb.setDirection(DcMotor.Direction.FORWARD);

            setupChassisMotor(lf);
            setupChassisMotor(rf);
            setupChassisMotor(lb);
            setupChassisMotor(rb);

            hubImu = hwMap.get(IMU.class, "imu");
            IMU.Parameters imuParameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            hubImu.initialize(imuParameters);
            hubImu.resetYaw();

            s1 = hwMap.get(DcMotorEx.class, "s1");
            s2 = hwMap.get(DcMotorEx.class, "s2");
            aservo1 = hwMap.get(Servo.class, "aservo1");
            aservo2 = hwMap.get(Servo.class, "aservo2");
            iservo1 = hwMap.get(Servo.class, "iservo1");
            iservo2 = hwMap.get(Servo.class, "iservo2");
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

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            s1.setPower(0);
            s2.setPower(0);
            intake.setPower(0);
            load.setPower(0);

            aservo1.setDirection(Servo.Direction.FORWARD);
            aservo2.setDirection(Servo.Direction.REVERSE);
            aservo1.setPosition(0.4);
            aservo2.setPosition(0.4);
        }

        private void setupChassisMotor(DcMotorEx motor) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CHASSIS_PIDF);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }
    }

    RobotHardware robot = new RobotHardware();

    // ===================================================================
    // ==========          二、多挡位独立目标/怠速及 PIDF 阵列          ==========
    // ===================================================================
    private int currentGear = 1;
    private double targetRPM = 1600.0;

    private final double GEAR_1_TARGET = 1550.0;
    private final double GEAR_1_IDLE   = 1300.0;

    private final double GEAR_2_TARGET = 1700.0;
    private final double GEAR_2_IDLE   = 1500.0;

    private final double GEAR_3_TARGET = 2000.0;
    private final double GEAR_3_IDLE   = 1700.0;

    private final double GEAR_4_TARGET = 2100.0;
    private final double GEAR_4_IDLE   = 1800.0;

    // 通用怠速 PIDF：调得稍微柔和点，专门负责松开 Trigger 时的平稳维持，绝不给满功率
    private final PIDFCoefficients PIDF_IDLE_COMMON = new PIDFCoefficients(13.2, 0, 0.5, 17.0);

    // 射击状态专属的高动态 PIDF 参数组
    private final PIDFCoefficients PIDF_GEAR_1 = new PIDFCoefficients(13.2, 0.0, 0.1, 22.6);
    private final PIDFCoefficients PIDF_GEAR_2 = new PIDFCoefficients(13.0, 0.0, 0.2, 22.5);
    private final PIDFCoefficients PIDF_GEAR_3 = new PIDFCoefficients(18.1, 0.0, 0.1, 21.6);
    private final PIDFCoefficients PIDF_GEAR_4 = new PIDFCoefficients(18.5, 0.0, 0.5, 22.0);

    // ===================================================================
    // ==========               核心状态与控制参数区               ==========
    // ===================================================================
    private double iCurrentPosition = 0.0;

    private final double BANGBANG_TRIGGER_THRESHOLD = 50.0;
    private final double RPM_TOLERANCE_LOWER = 30.0;
    private final double RPM_TOLERANCE_UPPER = 150.0;

    private boolean isFirstAcceleration = true;
    private boolean hasPassedThreshold = false;
    private ElapsedTime valveTimer = new ElapsedTime();
    private final double VALVE_SETTLE_DELAY_SEC = 0.5;
    private boolean isValveTimerReset = true;

    private final double STICK_DEADZONE = 0.08;
    private boolean lastAState = false;
    private boolean lastBState = false;

    private PIDFCoefficients activePIDF = null;

    // 面板可视化变量：用于记录当前帧两个电机到底被什么算法接管
    private String s1CoreDriverStatus = "STANDBY";
    private String s2CoreDriverStatus = "STANDBY";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        valveTimer.reset();

        applyShooterPIDF(PIDF_IDLE_COMMON);
        targetRPM = GEAR_1_TARGET;

        telemetry.addLine("32477: TeleOp_V3_newTest [可视化内核版] 已就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===================================================================
            // ==========      三、需求 4：底盘摇杆二次函数曲线映射 (Expo)   ==========
            // ===================================================================
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double rawTurn = gamepad1.right_stick_x;

            if (Math.abs(rawY) < STICK_DEADZONE) rawY = 0;
            if (Math.abs(rawX) < STICK_DEADZONE) rawX = 0;
            if (Math.abs(rawTurn) < STICK_DEADZONE) rawTurn = 0;

            double driveY = Math.signum(rawY) * (rawY * rawY);
            double driveX = Math.signum(rawX) * (rawX * rawX);
            double turn   = Math.signum(rawTurn) * (rawTurn * rawTurn);

            double lfPower = driveY + driveX + turn;
            double rfPower = driveY - driveX - turn;
            double lbPower = driveY - driveX + turn;
            double rbPower = driveY + driveX - turn;

            double maxChassisPower = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );
            if (maxChassisPower > 1.0) {
                lfPower /= maxChassisPower;
                rfPower /= maxChassisPower;
                lbPower /= maxChassisPower;
                rbPower /= maxChassisPower;
            }

            robot.lf.setPower(lfPower);
            robot.rf.setPower(rfPower);
            robot.lb.setPower(lbPower);
            robot.rb.setPower(rbPower);

            // ===================================================================
            // ==========             四、分挡切换及目标/怠速映射            ==========
            // ===================================================================
            if (gamepad1.dpad_down) {
                currentGear = 1;
                targetRPM = GEAR_1_TARGET;
                iCurrentPosition = 0.1;
            } else if (gamepad1.dpad_left) {
                currentGear = 2;
                targetRPM = GEAR_2_TARGET;
                iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_right) {
                currentGear = 3;
                targetRPM = GEAR_3_TARGET;
                iCurrentPosition = 0.5;
            } else if (gamepad1.dpad_up) {
                currentGear = 4;
                targetRPM = GEAR_4_TARGET;
                iCurrentPosition = 1.0;
            }

            // A/B 键无级微调
            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;
            if (currentAState && !lastAState) {
                targetRPM = Range.clip(targetRPM - 50.0, 0.0, 3000.0);
            }
            if (currentBState && !lastBState) {
                targetRPM = Range.clip(targetRPM + 50.0, 0.0, 3000.0);
            }
            lastAState = currentAState;
            lastBState = currentBState;

            // ===================================================================
            // ==========      五、飞轮核心闭环驱动内核（彻底杜绝怠速过冲）  ==========
            // ===================================================================
            boolean isTriggerPressed = gamepad1.right_trigger > 0.1;
            double currentLoopTargetRPM = 0;
            PIDFCoefficients expectedPIDF;

            PIDFCoefficients gearActivePIDF;
            double currentGearIdle;
            switch (currentGear) {
                case 2:  gearActivePIDF = PIDF_GEAR_2; currentGearIdle = GEAR_2_IDLE; break;
                case 3:  gearActivePIDF = PIDF_GEAR_3; currentGearIdle = GEAR_3_IDLE; break;
                case 4:  gearActivePIDF = PIDF_GEAR_4; currentGearIdle = GEAR_4_IDLE; break;
                default: gearActivePIDF = PIDF_GEAR_1; currentGearIdle = GEAR_1_IDLE; break;
            }

            // 获取真实硬件转速 (RPM)
            double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
            double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

            if (isTriggerPressed) {
                // =============== 1. 射击模式 (激活 Bang-Bang 补速机制) ===============
                currentLoopTargetRPM = targetRPM;
                expectedPIDF = gearActivePIDF;

                double targetTicksPerSec = (currentLoopTargetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;
                double currentError1 = currentLoopTargetRPM - actualRPM1;
                double currentError2 = currentLoopTargetRPM - actualRPM2;

                // 首次拉起判断：当两台电机越过目标线或逼近目标线（差15圈以内），解开初次加速锁
                if (isFirstAcceleration) {
                    if (actualRPM1 >= (targetRPM - 15.0) && actualRPM2 >= (targetRPM - 15.0)) {
                        isFirstAcceleration = false;
                    }
                }

                // 驱动轮 1 复合决策与看板上报
                if (isFirstAcceleration) {
                    robot.s1.setPower(1.0);
                    s1CoreDriverStatus = "🔥 BANG-BANG [初次暴力起旋]";
                } else if (currentError1 >= BANGBANG_TRIGGER_THRESHOLD) {
                    robot.s1.setPower(1.0);
                    s1CoreDriverStatus = "⚡ BANG-BANG [吃球失速强推补偿]";
                } else {
                    robot.s1.setVelocity(targetTicksPerSec);
                    s1CoreDriverStatus = "⚙️ PIDF [精准转速闭环控制中]";
                }

                // 驱动轮 2 复合决策与看板上报
                if (isFirstAcceleration) {
                    robot.s2.setPower(1.0);
                    s2CoreDriverStatus = "🔥 BANG-BANG [初次暴力起旋]";
                } else if (currentError2 >= BANGBANG_TRIGGER_THRESHOLD) {
                    robot.s2.setPower(1.0);
                    s2CoreDriverStatus = "⚡ BANG-BANG [吃球失速强推补偿]";
                } else {
                    robot.s2.setVelocity(targetTicksPerSec);
                    s2CoreDriverStatus = "⚙️ PIDF [精准转速闭环控制中]";
                }

            } else {
                // =============== 2. 怠速模式 (绝对禁止 Bang-Bang，100% 隔离) ===============
                currentLoopTargetRPM = currentGearIdle;
                expectedPIDF = PIDF_IDLE_COMMON;

                double targetTicksPerSec = (currentLoopTargetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;

                // 怠速模式下只准使用内置 PIDF 控制，直接切断 setPower(1.0) 的任何可能
                robot.s1.setVelocity(targetTicksPerSec);
                robot.s2.setVelocity(targetTicksPerSec);

                s1CoreDriverStatus = "💤 PIDF [纯怠速维持 - 严禁强推]";
                s2CoreDriverStatus = "💤 PIDF [纯怠速维持 - 严禁强推]";

                // 强行锁死并重置所有射击流触发器
                isFirstAcceleration = true;
                hasPassedThreshold = false;
                isValveTimerReset = true;
            }

            // 按需安全注入参数
            if (activePIDF != expectedPIDF) {
                applyShooterPIDF(expectedPIDF);
            }

            // ===================================================================
            // ==========             六、防走火门与球道放行控制               ==========
            // ===================================================================
            boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);
            boolean isSpeedNowReady = isTriggerPressed && s1SpeedReady && s2SpeedReady;

            if (isTriggerPressed && !hasPassedThreshold && isSpeedNowReady) {
                hasPassedThreshold = true;
            }

            boolean isBallPathReadyToRelease = false;

            if (hasPassedThreshold) {
                robot.aservo1.setPosition(0.0);
                robot.aservo2.setPosition(0.0);
                if (isValveTimerReset) {
                    valveTimer.reset();
                    isValveTimerReset = false;
                }
                if (valveTimer.seconds() >= VALVE_SETTLE_DELAY_SEC) {
                    isBallPathReadyToRelease = true;
                }
            } else {
                robot.aservo1.setPosition(0.4);
                robot.aservo2.setPosition(0.4);
                isValveTimerReset = true;
            }

            // 俯仰角控制
            if (gamepad1.x) iCurrentPosition = Range.clip(iCurrentPosition + 0.005, 0.0, 1.0);
            if (gamepad1.y) iCurrentPosition = Range.clip(iCurrentPosition - 0.005, 0.0, 1.0);
            robot.iservo1.setPosition(iCurrentPosition);
            robot.iservo2.setPosition(1.0 - iCurrentPosition);

            // 球道及吸球动力管理
            String ballTrackStatus = "IDLE";
            double intakePower = 0.0;
            double loadPower = 0.0;

            if (gamepad1.left_trigger > 0.1) {
                if (!hasPassedThreshold) {
                    intakePower = 0.9;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [WAITING SPEED]";
                } else if (!isBallPathReadyToRelease) {
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = "LT [VALVE OPENING]";
                } else if (!isSpeedNowReady) {
                    intakePower = 0.0;
                    loadPower = 0.0;
                    ballTrackStatus = "⚠️ LT [RPM DROPPED INTERCEPT]";
                } else {
                    intakePower = 0.9;
                    loadPower = 0.9;
                    ballTrackStatus = "LT [FIRE]";
                }
            } else {
                if (gamepad1.right_bumper) {
                    intakePower = 0.9;
                    loadPower = -0.9;
                    ballTrackStatus = "RB [BALANCED ACCUMULATION]";
                } else if (gamepad1.back) {
                    intakePower = -0.9;
                    loadPower = -0.9;
                    ballTrackStatus = "💥 EMERGENCY REVERSE";
                } else {
                    intakePower = 0.2;
                    loadPower = 0.0;
                    ballTrackStatus = "⚡ KEEP BALANCE";
                }
            }

            robot.intake.setPower(intakePower);
            robot.load.setPower(loadPower);

            // ===================================================================
            // ==========          七、数据看板：可视化驱动内核监控          ==========
            // ===================================================================
            telemetry.addLine("============ 32477 DRIVER PANEL ============");
            telemetry.addData("Current Gear", "Gear %d (Target: %.0f RPM)", currentGear, targetRPM);
            telemetry.addData("System Loop Mode", isTriggerPressed ? "🔥 SHOOT FLOW" : "💤 STANDBY IDLE");
            telemetry.addData("Target Speed (This Frame)", "%.0f RPM", currentLoopTargetRPM);
            telemetry.addLine("--------------------------------------------");

            // 核心监控：两路电机当前的物理内核状态一目了然
            telemetry.addLine("[MOTOR 1 KERNEL MONITOR]");
            telemetry.addData(" -> Real Velocity", "%.1f RPM", actualRPM1);
            telemetry.addData(" -> Real Error", "%.1f RPM", (currentLoopTargetRPM - actualRPM1));
            telemetry.addData(" -> ACTIVE DRIVER", s1CoreDriverStatus); // 显式看清是否触发了 Bang-Bang
            telemetry.addLine("--------------------------------------------");

            telemetry.addLine("[MOTOR 2 KERNEL MONITOR]");
            telemetry.addData(" -> Real Velocity", "%.1f RPM", actualRPM2);
            telemetry.addData(" -> Real Error", "%.1f RPM", (currentLoopTargetRPM - actualRPM2));
            telemetry.addData(" -> ACTIVE DRIVER", s2CoreDriverStatus); // 显式看清是否触发了 Bang-Bang
            telemetry.addLine("--------------------------------------------");

            telemetry.addData("Ball Track Status", ballTrackStatus);
            telemetry.update();
        }
    }

    private void applyShooterPIDF(PIDFCoefficients pidf) {
        if (robot.s1 != null && robot.s2 != null) {
            robot.s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            robot.s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            activePIDF = pidf;
        }
    }
}