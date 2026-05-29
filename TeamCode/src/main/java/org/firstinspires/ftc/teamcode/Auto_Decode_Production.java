package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// 修正后的 Pedro Pathing v2.x 核心包引用
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_Decode_Production", group = "Production")
public class Auto_Decode_Production extends OpMode {

    private RobotHardwareV3 robot;
    private IntakeController intakeManager;
    private ShooterController shooterManager;
    private Follower follower;

    private ElapsedTime timer;
    private int state;

    // 预先使用新版 Pose 定义所有路径关键点（X, Y, Heading以弧度为单位）
    private final Pose startPose = new Pose(20.25, 119.5, Math.toRadians(180));
    private final Pose shootPose = new Pose(40.0, 101.0, Math.toRadians(135));
    private final Pose midControlPose = new Pose(57.8437, 83.5912, 0); // 贝塞尔控制点，角度不参与运算
    private final Pose midPose = new Pose(46.6506, 82.5836, Math.toRadians(180));
    private final Pose endPose = new Pose(15.952, 82.574, Math.toRadians(180));

    private Path path1;
    private PathChain path2And3;

    @Override
    public void init() {
        // 1. 初始化硬件和子系统控制器
        robot = new RobotHardwareV3();
        robot.init(hardwareMap);

        intakeManager = new IntakeController(robot);
        shooterManager = new ShooterController(robot);

        // 2. 使用 Constants 组装工厂初始化 Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 3. 构建路径 1：前往发射点（直线）
        path1 = new Path(new BezierLine(startPose, shootPose));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // 4. 构建路径 2 和 3：前往终点并拼接为 PathChain
        // 路径 2 为二阶贝塞尔曲线段（传入起点、控制点、终点）
        Path segment2 = new Path(new BezierCurve(shootPose, midControlPose, midPose));
        segment2.setLinearHeadingInterpolation(shootPose.getHeading(), midPose.getHeading());

        // 路径 3 为直线段
        Path segment3 = new Path(new BezierLine(midPose, endPose));
        segment3.setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading());

        path2And3 = follower.pathBuilder()
                .addPath(segment2)
                .addPath(segment3)
                .build();

        timer = new ElapsedTime();
        state = 0;

        telemetry.addLine("Auto Initialized Successfully.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        follower.followPath(path1, true);
        state = 1;
    }

    @Override
    public void loop() {
        // 核心：底盘追踪每帧高频刷新
        follower.update();

        switch (state) {
            case 1:
                // 状态 1：前往发射点，同步启动飞轮至4档（推弹门保持关闭）
                shooterManager.updateShooter(4, true, false);
                intakeManager.stopOrLock(false);

                if (!follower.isBusy()) {
                    timer.reset();
                    state = 2; // 到达位置，进入发射状态
                }
                break;

            case 2:
                // 状态 2：静止发射，维持 3 秒（打开大门全力推弹）
                shooterManager.updateShooter(4, true, true);
                intakeManager.stopOrLock(false);

                if (timer.seconds() >= 3.0) {
                    // 发射 3 秒结束，飞轮恢复怠速，启动吸球，继续移动
                    shooterManager.updateShooter(4, false, false);
                    intakeManager.runIntakeMode();

                    follower.followPath(path2And3, true);
                    timer.reset();
                    state = 3;
                }
                break;

            case 3:
                // 状态 3：向终点移动，过程中维持怠速和吸球
                shooterManager.updateShooter(4, false, false);
                intakeManager.runIntakeMode();

                if (!follower.isBusy()) {
                    timer.reset();
                    state = 4; // 到达终点，进入静止等待
                }
                break;

            case 4:
                // 状态 4：在终点静止维持 3 秒
                shooterManager.updateShooter(4, false, false);
                intakeManager.runIntakeMode();

                if (timer.seconds() >= 3.0) {
                    // 锁死/关闭吸球
                    intakeManager.stopOrLock(false);
                    state = 5;
                }
                break;

            case 5:
                // 状态 5：结束程序
                requestOpModeStop();
                break;
        }

        // 数据反馈
        telemetry.addData("Current State", state);
        telemetry.addData("X Pose", "%.3f", follower.getPose().getX());
        telemetry.addData("Y Pose", "%.3f", follower.getPose().getY());
        telemetry.addData("Heading Deg", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}