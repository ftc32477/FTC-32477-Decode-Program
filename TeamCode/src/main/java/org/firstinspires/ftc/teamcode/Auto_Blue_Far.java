package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_Blue_Far", group = "Autonomous")
public class Auto_Blue_Far extends LinearOpMode {

    public Follower follower;
    // 使用自动程序专属硬件映射类，保持机构与底盘解耦
    private RobotHardwareV3_Auto robot = new RobotHardwareV3_Auto();

    // 机构子系统控制器声明
    private IntakeController intakeController;
    private ShooterController shooterController;

    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // 路径点判定到达的公差范围（英寸）
    private final double POS_TOLERANCE = 2.0;

    // =============================================================================
    // 🎛️ 机构异步控制底层状态旗标（与 Near 对应关系完全一致）
    // =============================================================================
    private int shooterGear = 2;
    private boolean requestSpinUp = false;
    private boolean requestFire = false;
    private boolean requestIntake = false;

    // =============================================================================
    // 📍 严格提取自 Auto_Blue_Far.pp 文件的真实路径坐标点配置
    // =============================================================================
    private final Pose startPose = new Pose(54.7, 8.8,  Math.toRadians(90.0));
    private final Pose pose1     = new Pose(60.0, 12.0, Math.toRadians(120.0));
    private final Pose pose2     = new Pose(32.8, 6.7,  Math.toRadians(180.0));
    private final Pose pose3     = new Pose(10.0, 6.7,  Math.toRadians(180.0));
    private final Pose pose4     = new Pose(15.0, 6.7,  Math.toRadians(180.0));
    private final Pose pose7     = new Pose(10.0, 6.7,  Math.toRadians(180.0));
    private final Pose pose8     = new Pose(60.0, 12.0, Math.toRadians(120.0));
    private final Pose pose9     = new Pose(6.7,  8.8,  Math.toRadians(90.0));

    // =============================================================================
    // 🗺️ 严格对应 .pp 顺序拆解的 7 段最小单位原子路径链
    // =============================================================================
    private PathChain path1, path2, path3, path4, path7, path8, path9;

    @Override
    public void runOpMode() {
        // 初始化底盘与全车硬件接口
        robot.init(hardwareMap);

        // ===================================================================
        // ✨【核心修复点】同步 Near 的舵机方向修正与初姿硬锁紧，规避卡死
        // ===================================================================
        if (robot.aservo1 != null) {
            robot.aservo1.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
            robot.aservo1.setPosition(0.4); // 默认拦截闭合位
        }
        if (robot.aservo2 != null) {
            robot.aservo2.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
            robot.aservo2.setPosition(0.4);
        }
        if (robot.iservo1 != null) {
            robot.iservo1.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
            robot.iservo1.setPosition(0.5);
        }
        if (robot.iservo2 != null) {
            robot.iservo2.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
            robot.iservo2.setPosition(0.5);
        }

        // 实例化解耦后的上层核心机构控制器
        intakeController = new IntakeController(robot);
        shooterController = new ShooterController(robot);

        // 绑定 Constants 测量出的高精度 PIDF 参数与跟随器
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 构建最小单位路径链
        buildGranularPaths();

        // 在等待发车期间（Init Loop）持续刷新锁紧舵机，确保开局状态完美
        while (!isStarted() && !isStopRequested()) {
            shooterController.updateShooter(2, false, false);
            telemetry.addLine("📌 【远端蓝方】32477 状态机就绪，舵机流向已硬锁紧...");
            telemetry.addData("闸门状态", shooterController.getShooterStatusStr());
            telemetry.update();
        }

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update(); // 刷新里程计解算与底盘马达电压输出

            autonomousChassisUpdate(); // 驱动远端专属原子状态机

            // ===================================================================
            // 🔄 异步执行上层机构控制（不使用 sleep 抢占 CPU 时间）
            // ===================================================================
            if (requestIntake) {
                intakeController.runIntakeMode();
            } else {
                if (!requestFire) {
                    intakeController.stopOrLock(false);
                }
            }

            // 实时刷新飞轮 PIDF 环路控制及舵机发射时序
            shooterController.updateShooter(shooterGear, requestSpinUp, requestFire);

            // ===================================================================
            // 🎯 实时路径颗粒化与机构状态双重看板（Telemetry）
            // ===================================================================
            String currentStage = "未知状态";
            String targetPointName = "无";
            Pose targetPose = follower.getPose();
            boolean isWaitingState = false;

            switch (pathState) {
                case 0:  currentStage = "准备发车"; targetPointName = "起点"; targetPose = startPose; break;
                case 1:  currentStage = "正在执行 Path 1 (前往首发点)"; targetPointName = "pose1"; targetPose = pose1; break;
                case 2:  currentStage = "💥 开放球道：第 1 次发射中 (等待3秒)"; targetPointName = "原地静止"; targetPose = pose1; isWaitingState = true; break;
                case 3:  currentStage = "正在执行 Path 2 (前往推球起点)"; targetPointName = "pose2"; targetPose = pose2; break;
                case 4:  currentStage = "正在执行 Path 3 (前方推球)"; targetPointName = "pose3"; targetPose = pose3; break;
                case 5:  currentStage = "正在执行 Path 4 (后退拉回)"; targetPointName = "pose4"; targetPose = pose4; break;
                case 6:  currentStage = "正在执行 Path 7 (二次推球)"; targetPointName = "pose7"; targetPose = pose7; break;
                case 7:  currentStage = "正在执行 Path 8 (返回首发点)"; targetPointName = "pose8"; targetPose = pose8; break;
                case 8:  currentStage = "💥 开放球道：第 2 次发射中 (等待3秒)"; targetPointName = "原地静止"; targetPose = pose8; isWaitingState = true; break;
                case 9:  currentStage = "正在执行 Path 9 (最终收尾冲刺停靠)"; targetPointName = "pose9"; targetPose = pose9; break;
                default: currentStage = "🏁 自动轨迹与发射流程安全结束"; targetPointName = "终点"; targetPose = pose9; break;
            }

            Pose currentPose = follower.getPose();
            double distanceError = Math.hypot(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

            telemetry.addLine("============ 📍 32477 里程计真实定位 ============");
            telemetry.addData("真实坐标 X", "%.2f 英寸", currentPose.getX());
            telemetry.addData("真实坐标 Y", "%.2f 英寸", currentPose.getY());
            telemetry.addData("真实朝向 Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            telemetry.addLine("\n============ 🎛️ 机构闭环耦合反馈 ============");
            telemetry.addData("飞轮请求/档位", "%s (Gear %d)", requestSpinUp ? "起旋中" : "怠速", shooterGear);
            telemetry.addData("推弹开闸信号", requestFire ? "开闸 (FIRE)" : "拦截 (HOLD)");
            telemetry.addData("球道状态", shooterController.getShooterStatusStr());
            telemetry.addData("吸球状态", intakeController.intakeStatus);

            telemetry.addLine("\n============ 🎯 状态机流转看板 ============");
            telemetry.addData("当前状态编码", "State [%d]", pathState);
            telemetry.addData("当前任务描述", currentStage);
            if (isWaitingState) {
                telemetry.addData("⏱️ 静态保留时间", "%.1f / 3.0 秒", stateTimer.seconds());
            } else {
                telemetry.addData("📏 单步位移误差", "%.2f 英寸", distanceError);
            }

            telemetry.update();
        }
    }

    // =============================================================================
    // 🛠️ 原子路径构建函数：精准复刻 .pp 文件定义的所有转向与逆向属性
    // =============================================================================
    public void buildGranularPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        // 依据 .pp 蓝本，以下直线路段采用切线/恒定朝向进行高效率推球吸球
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setConstantHeadingInterpolation(pose2.getHeading())
                .build();

        // Path 4 严格配置为逆向（reverse: true）拉回车辆
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setConstantHeadingInterpolation(pose3.getHeading())
                .setReversed()
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose7))
                .setConstantHeadingInterpolation(pose4.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();
    }

    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }

    // =============================================================================
    // 🔄 严格按 .pp 序列流转的紧凑型自动状态机
    // =============================================================================
    public void autonomousChassisUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1: // 运行 Path 1 前往首发点
                shooterGear = 4;
                requestSpinUp = true;   // 提前把飞轮拉满起旋
                requestFire = false;
                requestIntake = false;
                if (hasReached(pose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 精确触发第 1 次 3 秒静止停留
                    pathState = 2;
                }
                break;

            case 2: // 💥 第一次设球（对应序列中 Path 1 后的 3000ms 挂起）
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开闸射球
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    requestFire = false;
                    follower.followPath(path2); // 射球完毕，立刻发车前往推球区
                    pathState = 3;
                }
                break;

            case 3: // 运行 Path 2
                shooterGear = 4;
                requestSpinUp = false;  // 飞轮切回怠速
                requestFire = false;
                requestIntake = true;   // 开启 Intake 吸球/推球
                if (hasReached(pose2) || !follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4: // 运行 Path 3 持续向前推球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(pose3) || !follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 5;
                }
                break;

            case 5: // 运行 Path 4 倒车拉回
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(pose4) || !follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 6;
                }
                break;

            case 6: // 运行 Path 7 二次向前推球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(pose7) || !follower.isBusy()) {
                    follower.followPath(path8);
                    pathState = 7;
                }
                break;

            case 7: // 运行 Path 8 返回第二发射点（pose8）
                shooterGear = 4;
                requestSpinUp = true;   // 重新拉高飞轮转速准备开火
                requestFire = false;
                requestIntake = false;  // 停止常态吸球
                if (hasReached(pose8) || !follower.isBusy()) {
                    stateTimer.reset(); // 精确触发第 2 次 3 秒静止停留
                    pathState = 8;
                }
                break;

            case 8: // 💥 第二次设球（对应序列中 Path 8 后的 3000ms 挂起）
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开闸射球
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    requestFire = false;
                    follower.followPath(path9); // 射球完毕，立刻进行最终收尾冲刺
                    pathState = 9;
                }
                break;

            case 9: // 运行 Path 9 前往终点停靠
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;   // 开启常态吸球兜底
                if (hasReached(pose9) || !follower.isBusy()) {
                    pathState = -1;     // 自动流程安全结束
                }
                break;

            default: // 安全兜底重置
                shooterGear = 2;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = false;
                break;
        }
    }
}