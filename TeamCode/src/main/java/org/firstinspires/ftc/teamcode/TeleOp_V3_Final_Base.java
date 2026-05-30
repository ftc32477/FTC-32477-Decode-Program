package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * 32477Origin 手动控制基类 - 统一滤层编译修复版
 */
public class TeleOp_V3_Final_Base extends LinearOpMode {

    // ========== 1. 核心控制器声明 ==========
    protected RobotHardwareV3 robot = new RobotHardwareV3();
    protected IntakeController intakeManager;
    protected ShooterController shooterManager;

    // ========== 2. 状态机与遥控参数 ==========
    private int currentGear = 1;
    private final double STICK_DEADZONE = 0.08;

    private int driveMode = 0;
    private boolean lastLBState = false;
    private boolean systemActivated = false;

    // 无头模式控制旗标（默认开启）
    protected boolean isFieldCentric = true;
    private boolean lastOptionsState = false;

    // 操作手视角的偏航角软件补偿（红蓝方子程序独立配置）
    protected double angleOffset = 0.0;

    protected double targetAngleA = 0.0;
    protected double targetAngleB = 0.0;

    @Override
    public void runOpMode() {
        // 🛠️ 1. 初始化硬件
        robot.init(hardwareMap);

        // 🛠️ 2. 实例化子系统控制器
        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        telemetry.addData("Status", "🔥 纯净调用滤层配置成功，等待开始...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =====================================================================
            // 🚀 核心对齐点 1：高频刷新 Pedro 跟随器
            // =====================================================================
            if (robot.follower != null) {
                robot.follower.update();
            }

            // 🛠️ 【已修复】改用 Pedro Pathing 原生追踪算法获取当前无偏航高精度航向角（弧度转角度）
            double odoHeading = 0.0;
            if (robot.follower != null) {
                odoHeading = Math.toDegrees(robot.follower.getPose().getHeading());
            }

            // --- 摇杆原始输入捕捉 ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // 死区过滤
            if (Math.abs(y) < STICK_DEADZONE) y = 0;
            if (Math.abs(x) < STICK_DEADZONE) x = 0;
            if (Math.abs(rx) < STICK_DEADZONE) rx = 0;

            // 摇杆响应平方平滑映射
            double driveY = Math.signum(y) * y * y;
            double driveX = Math.signum(x) * x * x;
            double turn = Math.signum(rx) * rx * rx;

            // --- 遥控快慢档位控制 ---
            if (gamepad1.dpad_up) currentGear = 3;
            else if (gamepad1.dpad_down) currentGear = 1;
            else if (gamepad1.dpad_left || gamepad1.dpad_right) currentGear = 2;

            double speedMultiplier = 0.7;
            if (currentGear == 1) speedMultiplier = 0.4;
            else if (currentGear == 2) speedMultiplier = 0.7;
            else if (currentGear == 3) speedMultiplier = 1.0;

            driveY *= speedMultiplier;
            driveX *= speedMultiplier;
            turn *= 0.6;

            // --- 自动瞄准 / 一键定向 P 闭环锁死逻辑 (A/B键) ---
            String targetLogStatus = "MANUAL CONTROL (自由驾驶)";
            if (gamepad1.a || gamepad1.b) {
                double targetLoc = gamepad1.a ? targetAngleA : targetAngleB;
                double compensatedHeading = normalizeAngle(odoHeading + angleOffset);
                double angleError = normalizeAngle(compensatedHeading - targetLoc);

                double P_COEFF = 0.035;
                turn = -angleError * P_COEFF;
                turn = Math.max(-0.4, Math.min(0.4, turn));
                targetLogStatus = gamepad1.a ? "🔒 LOCKING TARGET A" : "🔒 LOCKING TARGET B";
            }

            // --- 场体系 (无头模式) 切换边缘触发 ---
            if (gamepad1.options && !lastOptionsState) {
                isFieldCentric = !isFieldCentric;
            }
            lastOptionsState = gamepad1.options;

            // =====================================================================
            // 🚀 核心对齐点 2：完美的 setTeleOpDrive 手动滤层结合【已修复】
            // =====================================================================
            if (robot.follower != null) {
                if (isFieldCentric) {
                    // 融入红蓝方特定视角初始补偿 (angleOffset)，解算出绝对场体系物理旋转矩阵
                    double driverRelativeHeading = normalizeAngle(odoHeading + angleOffset);
                    double botHeadingRad = Math.toRadians(driverRelativeHeading);

                    double rotX = driveX * Math.cos(-botHeadingRad) - driveY * Math.sin(-botHeadingRad);
                    double rotY = driveX * Math.sin(-botHeadingRad) + driveY * Math.cos(-botHeadingRad);

                    // 🛠️【已修复】将 setTeleOpMovementVectors 替换为 Pedro 的 setTeleOpDrive
                    // 参数映射：rotY 是前后动力(X)，rotX 是左右横移(Y)，turn 是旋转
                    // 因为上方代码已经手动将坐标系转成了车体系，所以最后一项 robotCentric 传 true，防止二次旋转
                    robot.follower.setTeleOpDrive(rotY, rotX, turn, true);
                } else {
                    // 车体系直接注入
                    // 🛠️【已修复】将 setTeleOpMovementVectors 替换为 setTeleOpDrive
                    // 参数映射：driveY 是前后动力，driveX 是左右横移，最后一项传入 true (开启车体系)
                    robot.follower.setTeleOpDrive(driveY, driveX, turn, true);
                }
            }

            // =====================================================================
            // ==========     3. 上层机构业务状态机处理【接口完全对齐】     ==========
            // =====================================================================
            if (gamepad1.left_bumper && !lastLBState) {
                systemActivated = !systemActivated;
                driveMode = systemActivated ? 1 : 0;
            }
            lastLBState = gamepad1.left_bumper;

            if (driveMode == 0) {
                intakeManager.runIntakeMode();   // 激活吸球揉球
                shooterManager.runIdleMode();     // 【已修复】飞轮降速/大门重置
            } else {
                intakeManager.runHoldMode();     // 【已修复】锁球道停止吸取
                boolean requestFire = gamepad1.right_trigger > 0.15;
                shooterManager.runFireMode(requestFire); // 【已修复】状态机开火
            }

            // =====================================================================
            // ==========                4. 遥测面板输出                ==========
            // =====================================================================
            telemetry.addData("★ SYSTEM ACTIVE MODE", driveMode == 1 ? "🔥 SHOOTING" : "📥 INTAKE");
            telemetry.addData("★ CHASSIS DRIVE MODE", isFieldCentric ? "🌐 FIELD-CENTRIC" : "🤖 ROBOT-CENTRIC");
            telemetry.addData("Chassis Lock Status", targetLogStatus);
            telemetry.addData("-> Raw Heading", "%.2f °", odoHeading);
            telemetry.addData("-> Compensated Heading", "%.2f °", normalizeAngle(odoHeading + angleOffset));
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }
}