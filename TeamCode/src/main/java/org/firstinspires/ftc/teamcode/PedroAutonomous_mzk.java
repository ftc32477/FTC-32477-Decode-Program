package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroAutonomous_mzk", group = "Autonomous")
public class PedroAutonomous_mzk extends LinearOpMode {

    public Follower follower; // Pedro Pathing follower instance
    private RobotHardwareV3 robot = new RobotHardwareV3();

    // 引入子系统控制器
    private IntakeController intakeControl;
    private ShooterController shooterControl;

    private int pathState = 0; // Current autonomous path state (state machine)
    private ElapsedTime stateTimer = new ElapsedTime();

    // --- 机制控制核心旗标状态变量 ---
    private int targetShooterGear = 1;     // 默认飞轮档位 (1-4)
    private boolean spinUpRequested = false; // 是否请求起旋/怠速控制
    private boolean fireRequested = false;   // 是否请求开火/物理推弹
    private boolean intakeActive = false;    // 是否开启吸球球道

    // 专门用于边缘触发捕捉：记录上一帧是否正在全力喂弹
    private boolean wasFeedingLastFrame = false;

    // 严格照搬他成功走线所使用的几何距离误差判定阈值
    private final double POS_TOLERANCE = 2.0;

    // --- 【完全提取自你们最新发来的真实路径坐标点】 ---
    private final Pose startPose = new Pose(20.250, 119.500, Math.toRadians(143.0));
    private final Pose point2    = new Pose(40.000, 101.000, Math.toRadians(135.0));
    private final Pose point3    = new Pose(48.000, 82.500,  Math.toRadians(180.0));
    private final Pose point4    = new Pose(46.000, 82.500,  Math.toRadians(180.0));
    private final Pose endPose   = new Pose(15.000, 82.500,  Math.toRadians(180.0));

    // --- 严格分段独立路径声明 ---
    private PathChain pathSeg1, pathSeg2, pathSeg3, pathSeg4;

    @Override
    public void runOpMode() {
        // 1. 初始化硬件与解耦子系统
        robot.init(hardwareMap);
        intakeControl = new IntakeController(robot);
        shooterControl = new ShooterController(robot);

        // 2. 创建 Follower 并单次注入逻辑起点
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 3. 构建路径群
        buildPaths();

        telemetry.addData("Status", "PedroAutonomous_mzk (拉扯抖动修复版)已就绪");
        telemetry.update();

        waitForStart();

        stateTimer.reset();

        // 4. 主循环：完全复制他的结构，确保底盘、动作状态机每帧协同高频刷新
        while (opModeIsActive()) {
            follower.update();

            // 调度状态机函数（优先更新状态机，让 pathState 指引当前的机构分配权）
            autonomousPathUpdate();

            // --- 🔧 【核心修复区】：严格解耦并隔离控制权，杜绝两套状态机同时给推弹电机写功率 ---
            if (pathState == 2) {
                // 🚀 【阶段 A：射击点火阶段】
                // 此时控制权全权移交给 shooterControl。绝对不允许执行 intakeControl.stopOrLock()！
                shooterControl.updateShooter(targetShooterGear, spinUpRequested, fireRequested);
            } else {
                // 🛑 🏎️ 📥 【阶段 B：常规行驶或捡球阶段】
                // 此时 shooter 处于非开火请求状态，安全刷新飞轮即可
                shooterControl.updateShooter(targetShooterGear, spinUpRequested, fireRequested);

                // 刷新球道吸取/锁球状态
                if (intakeActive) {
                    intakeControl.runIntakeMode();   // 模式一：自动吸球
                } else {
                    intakeControl.stopOrLock(false); // 停止/常态静止锁球
                }
            }

            // ===================================================================
            // ==========                 双电机独立监控遥测                ==========
            // ===================================================================
            telemetry.addLine("============= 📊 SHOOTER MONITOR =============");
            telemetry.addData("当前目标档位", " Gear " + targetShooterGear);
            telemetry.addData("飞轮1真实转速 (s1)", "%.1f RPM", shooterControl.getShooter1RPM());
            telemetry.addData("飞轮2真实转速 (s2)", "%.1f RPM", shooterControl.getShooter2RPM());
            telemetry.addData("是否正在喂弹 (isFeeding)", shooterControl.isFeeding());
            telemetry.addData("掉速拦截状态 (isIntercepting)", shooterControl.isIntercepting());
            telemetry.addData("击发已维持时间", "%.2f 秒", stateTimer.seconds());
            telemetry.addData("当前底层轨道状态", shooterControl.getShooterStatusStr());

            telemetry.addLine("============= 🏎️ CHASSIS MONITOR =============");
            telemetry.addData("当前路径状态 (State)", pathState);
            telemetry.addData("坐标 X", "%.3f", follower.getPose().getX());
            telemetry.addData("坐标 Y", "%.3f", follower.getPose().getY());
            telemetry.addData("车头朝向 Heading°", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();

            // 滚动更新上一帧状态
            wasFeedingLastFrame = shooterControl.isFeeding();
        }
    }

    /**
     * 完全复制分段独立构建与角度插值风格
     */
    public void buildPaths() {
        pathSeg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, point2))
                .setLinearHeadingInterpolation(startPose.getHeading(), point2.getHeading())
                .build();

        pathSeg2 = follower.pathBuilder()
                .addPath(new BezierLine(point2, point3))
                .setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading())
                .build();

        pathSeg3 = follower.pathBuilder()
                .addPath(new BezierLine(point3, point4))
                .setLinearHeadingInterpolation(point3.getHeading(), point4.getHeading())
                .build();

        pathSeg4 = follower.pathBuilder()
                .addPath(new BezierLine(point4, endPose))
                .setConstantHeadingInterpolation(point4.getHeading())
                .build();
    }

    /**
     * 几何距离误差判定函数
     */
    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }

    /**
     * 状态机跳转逻辑
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = false;
                intakeActive = false;

                follower.followPath(pathSeg1); // 前往第一个射击点
                pathState = 1;
                break;

            case 1:
                // 检查是否到达第一个目的地（point2 射击点）
                if (hasReached(point2)) {
                    fireRequested = true; // 发出开火请求
                    pathState = 2;
                }
                break;

            case 2:
                // 维持发射请求
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = true;

                // 捕捉喂弹开始的瞬间，重置 6 秒计时
                if (shooterControl.isFeeding() && !wasFeedingLastFrame) {
                    stateTimer.reset();
                }

                // 在喂弹激活的状态下实打实满 6 秒放行
                if (shooterControl.isFeeding() && stateTimer.seconds() > 6.0) {
                    fireRequested = false;  // 关闭大门

                    // 战术切换：立刻切为4档大功率加速，为后半场蓄能
                    targetShooterGear = 4;
                    spinUpRequested = true;

                    follower.followPath(pathSeg2); // 发车前往 point3
                    pathState = 3;
                }
                break;

            case 3:
                // 检查是否到达 point3（即第二段结束）
                if (hasReached(point3)) {
                    // 后续路径吸球切入
                    intakeActive = true;

                    follower.followPath(pathSeg3); // 执行第 3 段短平移
                    pathState = 4;
                }
                break;

            case 4:
                // 检查是否到达 point4（即第三段结束）
                if (hasReached(point4)) {
                    follower.followPath(pathSeg4); // 执行第 4 段路径，全功率吸球平推
                    pathState = 5;
                }
                break;

            case 5:
                // 检查是否到达最终终点（endPose）
                if (hasReached(endPose)) {
                    intakeActive = false;    // 关闭吸球
                    spinUpRequested = false; // 关闭飞轮
                    pathState = -1;          // 自动结束
                }
                break;
        }
    }
}