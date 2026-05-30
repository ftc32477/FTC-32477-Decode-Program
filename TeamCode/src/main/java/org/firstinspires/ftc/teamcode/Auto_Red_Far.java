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

@Autonomous(name = "Auto_Red_Far", group = "Autonomous")
public class Auto_Red_Far extends LinearOpMode {

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
    // 📍 坐标点定义：直接精准替换为红方远端对应数据
    // =============================================================================
    private final Pose startPose = new Pose(89.3, 8.8,  Math.toRadians(90.0));
    private final Pose pose1     = new Pose(84.0, 12.0, Math.toRadians(60.0));
    private final Pose pose2     = new Pose(111.2, 6.7,  Math.toRadians(0.0));
    private final Pose pose3     = new Pose(134.0, 6.7,  Math.toRadians(0.0));
    private final Pose pose4     = new Pose(129.0, 6.7,  Math.toRadians(0.0));
    private final Pose pose7     = new Pose(134.0, 6.7,  Math.toRadians(0.0));
    private final Pose pose8     = new Pose(84.0, 12.0, Math.toRadians(60.0));
    private final Pose pose9     = new Pose(137.3,  8.8,  Math.toRadians(90.0));

    private PathChain path1, path2, path3, path4, path7, path8, path9;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose7))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose7.getHeading())
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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 运行 Path 1 前往第一发射点（pose1）
                shooterGear = 4;
                requestSpinUp = true;   // 提前热身飞轮
                requestFire = false;
                requestIntake = false;
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1: // 等待到达第一发射点
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = false;
                requestIntake = false;
                if (hasReached(pose1) || !follower.isBusy()) {
                    stateTimer.reset(); // 精确触发第 1 次 3 秒静止停留
                    pathState = 2;
                }
                break;

            case 2: // 💥 第一次射球（对应序列中 Path 1 后的 3000ms 挂起）
                shooterGear = 4;
                requestSpinUp = true;
                requestFire = true;     // 开闸射球
                requestIntake = false;
                if (stateTimer.seconds() > 3.0) {
                    requestFire = false;
                    follower.followPath(path2); // 射球完毕，立刻切入后续采集/推送路径
                    pathState = 3;
                }
                break;

            case 3: // 运行 Path 2
                shooterGear = 2;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;   // 开启常态吸球
                if (hasReached(pose2) || !follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4: // 运行 Path 3
                shooterGear = 2;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(pose3) || !follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 5;
                }
                break;

            case 5: // 运行 Path 4
                shooterGear = 2;
                requestSpinUp = false;
                requestFire = false;
                requestIntake = true;
                if (hasReached(pose4) || !follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 6;
                }
                break;

            case 6: // 运行 Path 7
                shooterGear = 2;
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

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);

        intakeController = new IntakeController(robot);
        shooterController = new ShooterController(robot);

        follower.setStartingPose(startPose);
        buildPaths();

        pathState = 0;

        telemetry.addData("Status", "Auto_Red_Far Initialized. Ready for Match!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        stateTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();

            if (requestIntake) {
                intakeController.runIntakeMode();
            } else {
                intakeController.runIdleMode();
            }

            shooterController.updateLogic(shooterGear, requestSpinUp, requestFire);

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(), follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }
}