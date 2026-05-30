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

@Autonomous(name = "Auto_Blue_Near_Chassis", group = "Autonomous")
public class Auto_Blue_Near_Chassis extends LinearOpMode {

    public Follower follower;
    private RobotHardwareV3 robot = new RobotHardwareV3(); // 仅保留底层硬件映射避免底盘报错

    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // 路径点判定到达的公差范围（英寸）
    private final double POS_TOLERANCE = 2.0;

    // =============================================================================
    // 📍 严格提取自 .pp 文件的基础坐标点配置
    // =============================================================================
    private final Pose startPose     = new Pose(20.250, 119.500, Math.toRadians(143.0));
    private final Pose shootPose1    = new Pose(40.000, 101.000, Math.toRadians(135.0));

    private final Pose p2_control    = new Pose(60.000, 82.000,  0);
    private final Pose p2_end        = new Pose(40.000, 82.000,  Math.toRadians(180.0));
    private final Pose p3_end        = new Pose(15.000, 82.000,  Math.toRadians(180.0));

    private final Pose p5_control    = new Pose(60.000, 60.000,  0);
    private final Pose p5_end        = new Pose(40.000, 60.000,  Math.toRadians(180.0));
    private final Pose p6_end        = new Pose(10.000, 60.000,  Math.toRadians(180.0));
    private final Pose p7_control    = new Pose(36.000, 48.000,  0);

    private final Pose p8_control    = new Pose(72.000, 36.000,  0);
    private final Pose p8_end        = new Pose(40.000, 36.000,  Math.toRadians(180.0));
    private final Pose p9_end        = new Pose(10.000, 36.000,  Math.toRadians(180.0));
    private final Pose waitPose2     = new Pose(60.000, 12.000,  Math.toRadians(117.0));

    private final Pose endPose       = new Pose(60.000, 36.000,  Math.toRadians(90.0));

    // =============================================================================
    // 🗺️ 拆分为 .pp 文件对应的最小原子路径单位
    // =============================================================================
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    @Override
    public void runOpMode() {
        // 初始化底盘所需的硬件接口
        robot.init(hardwareMap);

        // 🌟 直接引用并绑定 Constants 测量出的高精度 PIDF 参数与物理限制
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 构建最小单位路径链
        buildGranularPaths();

        telemetry.addLine("📌 【原子级纯底盘路径程序】解耦就绪，等待发车...");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update(); // 刷新里程计解算与底盘马达电压输出

            autonomousChassisUpdate(); // 驱动原子状态机

            // ===================================================================
            // 🎯 实时路径颗粒化解算看板（Telemetry）
            // ===================================================================
            String currentStage = "未知状态";
            String targetPointName = "无";
            Pose targetPose = follower.getPose();
            boolean isWaitingState = false;

            switch (pathState) {
                case 0:  currentStage = "准备发车"; targetPointName = "起点"; targetPose = startPose; break;
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
                default: currentStage = "🏁 纯底盘轨迹位移测试全部安全结束"; targetPointName = "终点"; targetPose = endPose; break;
            }

            Pose currentPose = follower.getPose();
            double distanceError = Math.hypot(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

            // 文本输出到司机站屏幕
            telemetry.addLine("============ 📍 32477 里程计真实定位 ============");
            telemetry.addData("真实坐标 X", "%.2f 英寸", currentPose.getX());
            telemetry.addData("真实坐标 Y", "%.2f 英寸", currentPose.getY());
            telemetry.addData("真实朝向 Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            telemetry.addLine("\n============ 🎯 .pp 原子单位路径实时汇报 ============");
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
    // 🛠️ 原子路径构建函数：严格拆解 .pp 文件中的最小路径单位，不进行任何合并
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
    // 🔄 原子级状态机流转控制（双重安全闭环判定：物理公差收敛 OR 底盘动力停止）
    // =============================================================================
    public void autonomousChassisUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1: // 追逐 shootPose1
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 1 次时间保留周期
                    pathState = 2;
                }
                break;

            case 2: // ⏱️ 精确保留 3 秒等待
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path2);
                    pathState = 3;
                }
                break;

            case 3: // 追逐 p2_end
                if (hasReached(p2_end) || !follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4: // 追逐 p3_end
                if (hasReached(p3_end) || !follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 5;
                }
                break;

            case 5: // 返回 shootPose1
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 2 次时间保留周期
                    pathState = 6;
                }
                break;

            case 6: // ⏱️ 精确保留 3 秒等待
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path5);
                    pathState = 7;
                }
                break;

            case 7: // 追逐 p5_end
                if (hasReached(p5_end) || !follower.isBusy()) {
                    follower.followPath(path6);
                    pathState = 8;
                }
                break;

            case 8: // 追逐 p6_end
                if (hasReached(p6_end) || !follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 9;
                }
                break;

            case 9: // 返回 shootPose1
                if (hasReached(shootPose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入第 3 次时间保留周期
                    pathState = 10;
                }
                break;

            case 10: // ⏱️ 精确保留 3 秒等待
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path8);
                    pathState = 11;
                }
                break;

            case 11: // 追逐 p8_end
                if (hasReached(p8_end) || !follower.isBusy()) {
                    follower.followPath(path9);
                    pathState = 12;
                }
                break;

            case 12: // 追逐 p9_end
                if (hasReached(p9_end) || !follower.isBusy()) {
                    follower.followPath(path10);
                    pathState = 13;
                }
                break;

            case 13: // 前往战术中转点 waitPose2
                if (hasReached(waitPose2) || !follower.isBusy()) {
                    stateTimer.reset(); // 进入中转停留周期
                    pathState = 14;
                }
                break;

            case 14: // ⏱️ 中转点停留 3 秒
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(path11);
                    pathState = 15;
                }
                break;

            case 15: // 前往终点 endPose
                if (hasReached(endPose) || !follower.isBusy()) {
                    pathState = -1; // 结束自动
                }
                break;
        }
    }
}