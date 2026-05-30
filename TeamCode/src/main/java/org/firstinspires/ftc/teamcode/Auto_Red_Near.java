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

@Autonomous(name = "Auto_Red_Near", group = "Autonomous")
public class Auto_Red_Near extends LinearOpMode {

    public Follower follower;
    // 更换为完整硬件映射类，以兼容 Intake 和 Shooter 控制器
    private RobotHardwareV3_Auto robot = new RobotHardwareV3_Auto();

    // 机构子系统控制器声明
    private IntakeController intakeController;
    private ShooterController shooterController;

    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // 路径点判定到达的公差范围（英寸）
    private final double POS_TOLERANCE = 2.0;

    // =============================================================================
    // 🎛️ 机构异步控制底层状态旗标（绝不阻塞底盘循环）
    // =============================================================================
    private int shooterGear = 2;
    private boolean requestSpinUp = false;
    private boolean requestFire = false;
    private boolean requestIntake = false;

    // =============================================================================
    // 📍 严格提取自 Auto_Blue_Near.pp 文件的基础坐标点配置
    // =============================================================================
    private final Pose startPose     = new Pose(123.750, 119.500, Math.toRadians(37.0));
    private final Pose shootPose1    = new Pose(104.000, 101.000, Math.toRadians(45.0));

    private final Pose p2_control    = new Pose(84.000, 82.000,  0);
    private final Pose p2_end        = new Pose(104.000, 82.000,  Math.toRadians(0.0));
    private final Pose p3_end        = new Pose(128.800, 82.000,  Math.toRadians(0.0));

    private final Pose p5_control    = new Pose(84.000, 60.000,  0);
    private final Pose p5_end        = new Pose(104.000, 60.000,  Math.toRadians(0.0));
    private final Pose p6_end        = new Pose(134.000, 60.000,  Math.toRadians(0.0));
    private final Pose p7_control    = new Pose(108.000, 48.000,  0);

    private final Pose p8_control    = new Pose(72.000, 36.000,  0);
    private final Pose p8_end        = new Pose(104.000, 36.000,  Math.toRadians(0.0));
    private final Pose p9_end        = new Pose(134.000, 36.000,  Math.toRadians(0.0));
    private final Pose waitPose2     = new Pose(84.000, 12.000,  Math.toRadians(63.0));

    private final Pose endPose       = new Pose(84.000, 36.000,  Math.toRadians(90.0));

    // =============================================================================
    // 🗺️ 拆分为 .pp 文件对应的最小原子路径单位
    // =============================================================================
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    @Override
    public void runOpMode() {
        // 初始化底盘与全车硬件接口
        robot.init(hardwareMap);

        // ===================================================================
        // ✨【核心修复点】显式修正并初始化 Shooter 所有舵机的方向与初姿
        // ===================================================================
        if (robot.aservo1 != null) {
            robot.aservo1.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
            robot.aservo1.setPosition(0.4); // 默认拦截闭合位
        }
        if (robot.aservo2 != null) {
            // 镜像安装必须物理反转，彻底解决其中一个舵机在前两次不打开或卡死的问题
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

        // 直接引用并绑定 Constants 测量出的高精度 PIDF 参数与物理限制
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 构建最小单位路径链
        buildGranularPaths();

        // ===================================================================
        // ⏱️ 在等待发车期间（Init Loop）持续刷新锁紧舵机，确保开局状态完美
        // ===================================================================
        while (!isStarted() && !isStopRequested()) {
            shooterController.updateShooter(2, false, false); // 强制使其保持在 WAITING FIRE TRIGGER 并压紧闸门
            telemetry.addLine("📌 【自动程序】32477 状态机就绪，舵机流向已修正并硬锁紧...");
            telemetry.addData("闸门状态", shooterController.getShooterStatusStr());
            telemetry.update();
        }

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update(); // 刷新里程计解算与底盘马达电压输出

            autonomousChassisUpdate(); // 驱动原子状态机，决策出当前的机构动作标志位

            // ===================================================================
            // 🔄 异步执行上层机构控制（绝不使用 sleep 抢占 CPU 时间）
            // ===================================================================
            if (requestIntake) {
                intakeController.runIntakeMode();
            } else {
                // 当 requestFire 为 true 时，ShooterController 内部会全权接管并重写吸球与推弹功率
                // 为了避免指令冲突，仅在不请求发射时释放吸球锁
                if (!requestFire) {
                    intakeController.stopOrLock(false);
                }
            }

            // 实时刷新飞轮 PIDF 环路控制及舵机时序
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
                case 1:  currentStage = "正在执行 Path 1 (前往首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 2:  currentStage = "💥 开放球道：第 1 次发射中"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 3:  currentStage = "正在执行 Path 2 (绕行推球点1)"; targetPointName = "p2_end"; targetPose = p2_end; break;
                case 4:  currentStage = "正在执行 Path 3 (直线下推球1)"; targetPointName = "p3_end"; targetPose = p3_end; break;
                case 5:  currentStage = "正在执行 Path 4 (从球区1返回首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 6:  currentStage = "💥 开放球道：第 2 次发射中"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 7:  currentStage = "正在执行 Path 5 (绕行推球点2)"; targetPointName = "p5_end"; targetPose = p5_end; break;
                case 8:  currentStage = "正在执行 Path 6 (直线下推球2)"; targetPointName = "p6_end"; targetPose = p6_end; break;
                case 9:  currentStage = "正在执行 Path 7 (从球区2曲线回首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 10: currentStage = "💥 开放球道：第 3 次发射中"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 11: currentStage = "正在执行 Path 8 (绕行推球点3)"; targetPointName = "p8_end"; targetPose = p8_end; break;
                case 12: currentStage = "正在执行 Path 9 (直线下推球3)"; targetPointName = "p9_end"; targetPose = p9_end; break;
                case 13: currentStage = "正在执行 Path 10 (开往底线战术中转点)"; targetPointName = "waitPose2"; targetPose = waitPose2; break;
                case 14: currentStage = "💥 开放球道：中转点战术发射中"; targetPointName = "原地中转"; targetPose = waitPose2; isWaitingState = true; break;
                case 15: currentStage = "正在执行 Path 11 (最终收尾冲刺停靠)"; targetPointName = "endPose"; targetPose = endPose; break;
                default: currentStage = "🏁 自动轨迹与发射流程安全结束"; targetPointName = "终点"; targetPose = endPose; break;
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
            telemetry.addData("真实轮速", "S1: %.1f | S2: %.1f RPM", shooterController.getShooter1RPM(), shooterController.getShooter2RPM());
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
    // 🛠️ 原子路径构建函数：严格拆解 .pp 文件中的最小路径单位
    // =============================================================================
    public void buildGranularPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p2_control, p2_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p2_end.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2_end, p3_end))
                .setConstantHeadingInterpolation(p2_end.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3_end, shootPose1))
                .setLinearHeadingInterpolation(p3_end.getHeading(), shootPose1.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p5_control, p5_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p5_end.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(p5_end, p6_end))
                .setConstantHeadingInterpolation(p5_end.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(p6_end, p7_control, shootPose1))
                .setLinearHeadingInterpolation(p6_end.getHeading(), shootPose1.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p8_control, p8_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p8_end.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(p8_end, p9_end))
                .setConstantHeadingInterpolation(p8_end.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(p9_end, waitPose2))
                .setLinearHeadingInterpolation(p9_end.getHeading(), waitPose2.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(waitPose2, endPose))
                .setLinearHeadingInterpolation(waitPose2.getHeading(), endPose.getHeading())
                .build();
    }

    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }

    // =============================================================================
    // 🔄 状态机流转控制（写入上层机构标志位判定）
    // =============================================================================
    public void autonomousChassisUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1: // 💥 对应车辆发车/前往首发点点位
                shooterGear = 4;
                requestSpinUp = true;   // 提前起旋飞轮至第二档
                requestFire = false;
                requestIntake = false;
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 1 次时间保留周期
                    pathState = 2;
                }
                break;

            case 2: // 💥 对应车辆静止第 1 周期：射球
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开启球道开闸发射
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path2);
                    pathState = 3;
                }
                break;

            case 3: // 💥 对应离开车辆暂停：切换怠速，开启常态吸球
                shooterGear = 4;
                requestSpinUp = false;  // 飞轮切换为怠速
                requestFire = false;
                requestIntake = true;   // 启动 Intake
                if (hasReached(p2_end) || !follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4: // 追逐 p3_end 保持推球和吸球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(p3_end) || !follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 5;
                }
                break;

            case 5: // 💥 对应车辆返回首发点点位
                shooterGear = 4;
                requestSpinUp = true;   // 再次拉起飞轮至第二档
                requestFire = false;
                requestIntake = false;  // 停止常态吸球准备开火
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 2 次时间保留周期
                    pathState = 6;
                }
                break;

            case 6: // 💥 对应车辆静止第 2 周期：射球
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开闸
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path5);
                    pathState = 7;
                }
                break;

            case 7: // 💥 对应离开车辆暂停：切换怠速，开启常态吸球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(p5_end) || !follower.isBusy()) {
                    follower.followPath(path6);
                    pathState = 8;
                }
                break;

            case 8: // 追逐 p6_end
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(p6_end) || !follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 9;
                }
                break;

            case 9: // 💥 对应车辆返回首发点点位
                shooterGear = 4;
                requestSpinUp = true;   // 拉起飞轮至第二档
                requestFire = false;
                requestIntake = false;
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 3 次时间保留周期
                    pathState = 10;
                }
                break;

            case 10: // 💥 对应车辆静止第 3 周期：射球
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;    // 开闸
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path8);
                    pathState = 11;
                }
                break;

            case 11: // 💥 对应离开车辆暂停：切换怠速，开启常态吸球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(p8_end) || !follower.isBusy()) {
                    follower.followPath(path9);
                    pathState = 12;
                }
                break;

            case 12: // 追逐 p9_end
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(p9_end) || !follower.isBusy()) {
                    follower.followPath(path10);
                    pathState = 13;
                }
                break;

            case 13: // 💥 对应车辆返回第二发射点 (waitPose2)
                shooterGear = 4;
                requestSpinUp = true;   // 飞轮升至第二档
                requestFire = false;
                requestIntake = false;
                if (hasReached(waitPose2) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入中转停留周期
                    pathState = 14;
                }
                break;

            case 14: // 💥 对应车辆静止第 4 周期：在中转点射球
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开闸
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path11);
                    pathState = 15;
                }
                break;

            case 15: // 💥 对应离开中转点：终点冲刺，切换怠速，启动吸球
                shooterGear = 4;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(endPose) || !follower.isBusy()) {
                    pathState = -1;     // 状态机正常结束标记
                }
                break;

            default: // 🏁 安全兜底重置
                shooterGear = 2;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = false;
                break;
        }
    }
}