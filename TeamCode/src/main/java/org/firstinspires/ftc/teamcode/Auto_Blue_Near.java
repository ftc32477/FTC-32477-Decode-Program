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

@Autonomous(name = "Auto_Blue_Near_Telemetry", group = "Autonomous")
public class Auto_Blue_Near extends LinearOpMode {

    public Follower follower; // Pedro Pathing 跟随器实例
    private RobotHardwareV3 robot = new RobotHardwareV3();

    // 子系统控制器
    private IntakeController intakeControl;
    private ShooterController shooterControl;

    private int pathState = 0; // 当前自动状态机状态
    private ElapsedTime stateTimer = new ElapsedTime();

    // --- 机制控制核心旗标状态变量 ---
    private int targetShooterGear = 1;
    private boolean spinUpRequested = false;
    private boolean fireRequested = false;
    private boolean intakeActive = false;
    private boolean wasFeedingLastFrame = false;

    private final double POS_TOLERANCE = 2.0; // 判定到达终点的公差范围（英寸）

    // --- 【Auto_Blue_Near 路径点坐标配置】 ---
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

    private PathChain pathSeg1, pathSeg2_3_4, pathSeg5_6_7, pathSeg8_9_10, pathSeg11;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intakeControl = new IntakeController(robot);
        shooterControl = new ShooterController(robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addLine("📌 司机站独立高级文本监看系统就绪");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            follower.update();

            autonomousPathUpdate();

            // 子系统控制逻辑维持
            if (pathState == 2 || pathState == 4 || pathState == 6) {
                shooterControl.updateShooter(targetShooterGear, spinUpRequested, fireRequested);
            } else {
                shooterControl.updateShooter(targetShooterGear, spinUpRequested, fireRequested);
                if (intakeActive) {
                    intakeControl.runIntakeMode();
                } else {
                    intakeControl.stopOrLock(false);
                }
            }

            // ===================================================================
            // 🛠️ 【新需求：状态机路程阶段与目标点实时解算遥测】
            // ===================================================================
            String stageDescription;
            String targetPointName;
            String targetCoordinate;
            double distanceError = 0.0;
            Pose currentPose = follower.getPose(); // 获取当前小车真实定位

            switch (pathState) {
                case 0:
                    stageDescription = "状态 0: 准备出发阶段";
                    targetPointName = "首发投球点 (shootPose1)";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", shootPose1.getX(), shootPose1.getY(), Math.toDegrees(shootPose1.getHeading()));
                    break;
                case 1:
                    stageDescription = "状态 1: 正在开往首发投球点 [第一段路程]";
                    targetPointName = "首发投球点 (shootPose1)";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", shootPose1.getX(), shootPose1.getY(), Math.toDegrees(shootPose1.getHeading()));
                    distanceError = Math.hypot(currentPose.getX() - shootPose1.getX(), currentPose.getY() - shootPose1.getY());
                    break;
                case 2:
                    stageDescription = "状态 2: 已抵达首发点，正在执行：第 1 次高精度投球发射";
                    targetPointName = "原地推弹中 (等待 3.0 秒计时)";
                    targetCoordinate = "保持当前原位";
                    break;
                case 3:
                    stageDescription = "状态 3: 正在运行捡第一组样品并返回的路程 [第二段复合环线]";
                    targetPointName = "投球点 (shootPose1) [途径 p2_end -> p3_end]";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", shootPose1.getX(), shootPose1.getY(), Math.toDegrees(shootPose1.getHeading()));
                    distanceError = Math.hypot(currentPose.getX() - shootPose1.getX(), currentPose.getY() - shootPose1.getY());
                    break;
                case 4:
                    stageDescription = "状态 4: 已返回首发点，正在执行：第 2 次高精度投球发射";
                    targetPointName = "原地推弹中 (等待 3.0 秒计时)";
                    targetCoordinate = "保持当前原位";
                    break;
                case 5:
                    stageDescription = "状态 5: 正在运行捡第二组样品并返回的路程 [第三段复合环线]";
                    targetPointName = "投球点 (shootPose1) [途径 p5_end -> p6_end]";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", shootPose1.getX(), shootPose1.getY(), Math.toDegrees(shootPose1.getHeading()));
                    distanceError = Math.hypot(currentPose.getX() - shootPose1.getX(), currentPose.getY() - shootPose1.getY());
                    break;
                case 6:
                    stageDescription = "状态 6: 已返回首发点，正在执行：第 3 次高精度投球发射";
                    targetPointName = "原地推弹中 (等待 3.0 秒计时)";
                    targetCoordinate = "保持当前原位";
                    break;
                case 7:
                    stageDescription = "状态 7: 正在运行捡第三组样品并前往中转点的路程 [第四段中转路径]";
                    targetPointName = "底线中转等待点 (waitPose2)";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", waitPose2.getX(), waitPose2.getY(), Math.toDegrees(waitPose2.getHeading()));
                    distanceError = Math.hypot(currentPose.getX() - waitPose2.getX(), currentPose.getY() - waitPose2.getY());
                    break;
                case 8:
                    stageDescription = "状态 8: 已抵达中转点，正在执行：赛场战术静止等待";
                    targetPointName = "中转静止中 (等待 3.0 秒放行)";
                    targetCoordinate = "保持当前原位";
                    break;
                case 9:
                    stageDescription = "状态 9: 正在运行中转点开往最终停靠点的最终路程 [第五段收尾路径]";
                    targetPointName = "最终停靠终点 (endPose)";
                    targetCoordinate = String.format("X: %.1f, Y: %.1f, Heading: %.1f°", endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()));
                    distanceError = Math.hypot(currentPose.getX() - endPose.getX(), currentPose.getY() - endPose.getY());
                    break;
                default:
                    stageDescription = "自动程序正常结束";
                    targetPointName = "无目标";
                    targetCoordinate = "N/A";
                    break;
            }

            // --- 司机站文本刷新输出 ---
            telemetry.addLine("============ 📍 32477 里程计实时反馈 ============");
            telemetry.addData("当前坐标 X", "%.2f 英寸", currentPose.getX());
            telemetry.addData("当前坐标 Y", "%.2f 英寸", currentPose.getY());
            telemetry.addData("当前朝向 Heading°", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            telemetry.addLine("\n============ 🎯 运行阶段与目标监测 ============");
            telemetry.addData("【当前运行路程阶段】", stageDescription);
            telemetry.addData("【试图到达的目标点】", targetPointName);
            telemetry.addData("【目标点理论坐标值】", targetCoordinate);

            if (pathState == 1 || pathState == 3 || pathState == 5 || pathState == 7 || pathState == 9) {
                telemetry.addData("【到名义目标直线距离误差】", "%.2f 英寸", distanceError);
            } else {
                telemetry.addLine("【到名义目标直线距离误差】: 原地机制处理中...");
            }

            telemetry.addData("状态机内部计时", "%.1f 秒", stateTimer.seconds());
            telemetry.addData(" Pedro Follower 忙碌状态", follower.isBusy());

            telemetry.update();

            wasFeedingLastFrame = shooterControl.isFeeding();
        }
    }

    public void buildPaths() {
        // Path 1
        pathSeg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        // Path 2, 3, 4
        pathSeg2_3_4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p2_control, p2_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p2_end.getHeading())
                .addPath(new BezierLine(p2_end, p3_end))
                .setConstantHeadingInterpolation(p2_end.getHeading())
                .addPath(new BezierLine(p3_end, shootPose1))
                .setLinearHeadingInterpolation(p3_end.getHeading(), shootPose1.getHeading())
                .build();

        // Path 5, 6, 7
        pathSeg5_6_7 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p5_control, p5_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p5_end.getHeading())
                .addPath(new BezierLine(p5_end, p6_end))
                .setConstantHeadingInterpolation(p5_end.getHeading())
                .addPath(new BezierCurve(p6_end, p7_control, shootPose1))
                .setLinearHeadingInterpolation(p6_end.getHeading(), shootPose1.getHeading())
                .build();

        // Path 8, 9, 10
        pathSeg8_9_10 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, p8_control, p8_end))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), p8_end.getHeading())
                .addPath(new BezierLine(p8_end, p9_end))
                .setConstantHeadingInterpolation(p8_end.getHeading())
                .addPath(new BezierLine(p9_end, waitPose2))
                .setLinearHeadingInterpolation(p9_end.getHeading(), waitPose2.getHeading())
                .build();

        // Path 11
        pathSeg11 = follower.pathBuilder()
                .addPath(new BezierLine(waitPose2, endPose))
                .setLinearHeadingInterpolation(waitPose2.getHeading(), endPose.getHeading())
                .build();
    }

    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = false;
                intakeActive = false;

                follower.followPath(pathSeg1);
                pathState = 1;
                break;

            case 1:
                if (hasReached(shootPose1)) {
                    fireRequested = true;
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = true;

                if (shooterControl.isFeeding() && !wasFeedingLastFrame) {
                    stateTimer.reset();
                }

                if (stateTimer.seconds() > 3.0) {
                    fireRequested = false;
                    targetShooterGear = 4;
                    intakeActive = true;

                    follower.followPath(pathSeg2_3_4);
                    pathState = 3;
                }
                break;

            case 3:
                if (hasReached(shootPose1)) {
                    intakeActive = false;
                    fireRequested = true;
                    stateTimer.reset();
                    pathState = 4;
                }
                break;

            case 4:
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = true;

                if (shooterControl.isFeeding() && !wasFeedingLastFrame) {
                    stateTimer.reset();
                }

                if (stateTimer.seconds() > 3.0) {
                    fireRequested = false;
                    targetShooterGear = 4;
                    intakeActive = true;

                    follower.followPath(pathSeg5_6_7);
                    pathState = 5;
                }
                break;

            case 5:
                if (hasReached(shootPose1)) {
                    intakeActive = false;
                    fireRequested = true;
                    stateTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                targetShooterGear = 2;
                spinUpRequested = true;
                fireRequested = true;

                if (shooterControl.isFeeding() && !wasFeedingLastFrame) {
                    stateTimer.reset();
                }

                if (stateTimer.seconds() > 3.0) {
                    fireRequested = false;
                    targetShooterGear = 4;
                    intakeActive = true;

                    follower.followPath(pathSeg8_9_10);
                    pathState = 7;
                }
                break;

            case 7:
                if (hasReached(waitPose2)) {
                    intakeActive = false;
                    stateTimer.reset();
                    pathState = 8;
                }
                break;

            case 8:
                if (stateTimer.seconds() > 3.0) {
                    follower.followPath(pathSeg11);
                    pathState = 9;
                }
                break;

            case 9:
                if (hasReached(endPose)) {
                    intakeActive = false;
                    spinUpRequested = false;
                    pathState = -1;
                }
                break;
        }
    }
}