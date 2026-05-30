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

@Autonomous(name = "Auto_Red_Near_Chassis", group = "Autonomous")
public class Auto_Red_Near_Chassis extends LinearOpMode {

    public Follower follower;
    private RobotHardwareV3_Auto robot = new RobotHardwareV3_Auto(); // 仅保留底层硬件映射避免底盘报错

    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // 路径点判定到达的公差范围（英寸）
    private final double POS_TOLERANCE = 2.0;

    // =============================================================================
    // 📍 严格提取自 Auto_Red_Near.pp 文件的红方镜像坐标点配置
    // =============================================================================
    private final Pose startPose     = new Pose(123.750, 119.500, Math.toRadians(37.0));  // 144 - 20.25,  180 - 143
    private final Pose shootPose1    = new Pose(104.000, 101.000, Math.toRadians(45.0));  // 144 - 40.00,  180 - 135

    private final Pose p2_control    = new Pose(84.000,  82.000,  0);                     // 144 - 60.00
    private final Pose p2_end        = new Pose(104.000, 82.000,  Math.toRadians(0.0));   // 144 - 40.00,  180 - 180
    private final Pose p3_end        = new Pose(129.000, 82.000,  Math.toRadians(0.0));   // 144 - 15.00,  180 - 180

    private final Pose p5_control    = new Pose(84.000,  60.000,  0);                     // 144 - 60.00
    private final Pose p5_end        = new Pose(104.000, 60.000,  Math.toRadians(0.0));   // 144 - 40.00,  180 - 180
    private final Pose p6_end        = new Pose(134.000, 60.000,  Math.toRadians(0.0));   // 144 - 10.00,  180 - 180
    private final Pose p7_control    = new Pose(108.000, 48.000,  0);                     // 144 - 36.00

    private final Pose p8_control    = new Pose(72.000,  36.000,  0);                     // 144 - 72.00
    private final Pose p8_end        = new Pose(104.000, 36.000,  Math.toRadians(0.0));   // 144 - 40.00,  180 - 180
    private final Pose p9_end        = new Pose(134.000, 36.000,  Math.toRadians(0.0));   // 144 - 10.00,  180 - 180
    private final Pose waitPose2     = new Pose(84.000,  12.000,  Math.toRadians(63.0));  // 144 - 60.00,  180 - 117

    private final Pose endPose       = new Pose(84.000,  36.000,  Math.toRadians(90.0));  // 144 - 60.00,  180 - 90

    // =============================================================================
    // 🗺️ 拆分为 .pp 文件对应的红方最小原子路径单位
    // =============================================================================
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    @Override
    public void runOpMode() {
        // 初始化底盘所需的硬件接口
        robot.init(hardwareMap);

        // 直接引用并绑定 Constants 测量出的高精度 PIDF 参数与物理限制
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 构建红方原子路径链
        buildGranularPaths();

        telemetry.addLine("📌 【红方原子级纯底盘路径程序】镜像换算就绪...");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update(); // 刷新里程计解算与底盘马达电压输出

            autonomousChassisUpdate(); // 驱动原子状态机

            // ===================================================================
            // 🎯 红方实时路径颗粒化解算看板（Telemetry）
            // ===================================================================
            String currentStage = "未知状态";
            String targetPointName = "无";
            Pose targetPose = follower.getPose();
            boolean isWaitingState = false;

            switch (pathState) {
                case 0:  currentStage = "准备发车"; targetPointName = "红方起点"; targetPose = startPose; break;
                case 1:  currentStage = "正在执行 Path 1 (前往首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 2:  currentStage = "🛑 模拟第 1 次发射等待周期"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 3:  currentStage = "正在执行 Path 2 (绕行推球点1)"; targetPointName = "p2_end"; targetPose = p2_end; break;
                case 4:  currentStage = "正在执行 Path 3 (直线下推球1)"; targetPointName = "p3_end"; targetPose = p3_end; break;
                case 5:  currentStage = "正在执行 Path 4 (从球区1返回首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 6:  currentStage = "🛑 模拟第 2 次发射等待周期"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 7:  currentStage = "正在执行 Path 5 (绕行推球点2)"; targetPointName = "p5_end"; targetPose = p5_end; break;
                case 8:  currentStage = "正在执行 Path 6 (直线下推球2)"; targetPointName = "p6_end"; targetPose = p6_end; break;
                case 9:  currentStage = "正在执行 Path 7 (从球区2曲线回首发点)"; targetPointName = "shootPose1"; targetPose = shootPose1; break;
                case 10: currentStage = "🛑 模拟第 3 次发射等待周期"; targetPointName = "原地静止"; targetPose = shootPose1; isWaitingState = true; break;
                case 11: currentStage = "正在执行 Path 8 (绕行推球点3)"; targetPointName = "p8_end"; targetPose = p8_end; break;
                case 12: currentStage = "正在执行 Path 9 (直线下推球3)"; targetPointName = "p9_end"; targetPose = p9_end; break;
                case 13: currentStage = "正在执行 Path 10 (开往底线战术中转点)"; targetPointName = "waitPose2"; targetPose = waitPose2; break;
                case 14: currentStage = "🛑 模拟中转点静态战术等待"; targetPointName = "原地中转"; targetPose = waitPose2; isWaitingState = true; break;
                case 15: currentStage = "正在执行 Path 11 (最终收尾冲刺停靠)"; targetPointName = "endPose"; targetPose = endPose; break;
                default: currentStage = "🏁 红方纯底盘轨迹位移测试全部安全结束"; targetPointName = "终点"; targetPose = endPose; break;
            }

            Pose currentPose = follower.getPose();
            double distanceError = Math.hypot(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

            telemetry.addLine("============ 🔴 32477 红方里程计实时定位 ============");
            telemetry.addData("真实坐标 X", "%.2f 英寸", currentPose.getX());
            telemetry.addData("真实坐标 Y", "%.2f 英寸", currentPose.getY());
            telemetry.addData("真实朝向 Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            telemetry.addLine("\n============ 🎯 红方 .pp 原子单位路径实时汇报 ============");
            telemetry.addData("当前状态机编码", "State [%d]", pathState);
            telemetry.addData("当前运行段描述", currentStage);
            telemetry.addData("当前追逐目标点", targetPointName);
            telemetry.addData("目标理论坐标", "X: %.1f, Y: %.1f", targetPose.getX(), targetPose.getY());

            if (isWaitingState) {
                telemetry.addData("⏱️ 精确时间停留中", "%.1f / 3.0 秒", stateTimer.seconds());
            } else {
                telemetry.addData("📏 单步直线逼近误差", "%.2f 英寸", distanceError);
            }
            telemetry.addData("Pedro底盘物理动态", follower.isBusy() ? "奔跑中 (Busy)" : "目标收敛 (Idle)");

            telemetry.update();
        }
    }

    // =============================================================================
    // 🛠️ 原子路径构建函数：严格拆解红方 .pp 文件中的最小路径单位，不进行任何合并
    // =============================================================================
    public void buildGranularPaths() {

        // Path 1: 起点 -> 首发点
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        // Path 2: 首发点 -> 第1个推球点（曲线绕行）
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p2_control, p2_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p2_end.getHeading())
                .build();

        // Path 3: 第1个推球点 -> 推动结束点（直线平移）
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2_end, p3_end))
                .setConstantHeadingInterpolation(p2_end.getHeading())
                .build();

        // Path 4: 推动结束点1 -> 返回首发点
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3_end, shootPose1))
                .setLinearHeadingInterpolation(p3_end.getHeading(), shootPose1.getHeading())
                .build();

        // Path 5: 首发点 -> 第2个推球点（曲线绕行）
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p5_control, p5_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p5_end.getHeading())
                .build();

        // Path 6: 第2个推球点 -> 推动结束点（直线平移）
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(p5_end, p6_end))
                .setConstantHeadingInterpolation(p5_end.getHeading())
                .build();

        // Path 7: 推动结束点2 -> 曲线返回首发点
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(p6_end, p7_control, shootPose1))
                .setLinearHeadingInterpolation(p6_end.getHeading(), shootPose1.getHeading())
                .build();

        // Path 8: 首发点 -> 第3个推球点（曲线绕行）
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p8_control, p8_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p8_end.getHeading())
                .build();

        // Path 9: 第3个推球点 -> 推动结束点（直线平移）
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(p8_end, p9_end))
                .setConstantHeadingInterpolation(p8_end.getHeading())
                .build();

        // Path 10: 推动结束点3 -> 战术中转等待点
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(p9_end, waitPose2))
                .setLinearHeadingInterpolation(p9_end.getHeading(), waitPose2.getHeading())
                .build();

        // Path 11: 中转等待点 -> 最终停靠点
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
    // 🔄 原子级状态机流转控制
    // =============================================================================
    public void autonomousChassisUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1:
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (hasReached(p2_end) || !follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (hasReached(p3_end) || !follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 5;
                }
                break;

            case 5:
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path5);
                    pathState = 7;
                }
                break;

            case 7:
                if (hasReached(p5_end) || !follower.isBusy()) {
                    follower.followPath(path6);
                    pathState = 8;
                }
                break;

            case 8:
                if (hasReached(p6_end) || !follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 9;
                }
                break;

            case 9:
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path8);
                    pathState = 11;
                }
                break;

            case 11:
                if (hasReached(p8_end) || !follower.isBusy()) {
                    follower.followPath(path9);
                    pathState = 12;
                }
                break;

            case 12:
                if (hasReached(p9_end) || !follower.isBusy()) {
                    follower.followPath(path10);
                    pathState = 13;
                }
                break;

            case 13:
                if (hasReached(waitPose2) || !follower.isBusy()) {
                    stateTimer.reset();
                    pathState = 14;
                }
                break;

            case 14:
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path11);
                    pathState = 15;
                }
                break;

            case 15:
                if (hasReached(endPose) || !follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }
}