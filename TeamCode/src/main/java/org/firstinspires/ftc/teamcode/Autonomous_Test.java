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

@Autonomous(name = "Autonomous_Test_Fixed", group = "Production")
public class Autonomous_Test extends LinearOpMode {

    private Follower follower;
    private MechanismController mechanism;
    private RobotHardwareV3 robot = new RobotHardwareV3();

    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private final double POS_TOLERANCE = 2.0;

    // --- 坐标定义 ---
    private final Pose startPose = new Pose(25.663, 126.747, Math.toRadians(145));
    private final Pose shootPose = new Pose(41.915, 101.874, Math.toRadians(135));
    private final Pose controlPose = new Pose(59.679, 81.337); // 曲线控制点
    private final Pose intakeStartPose = new Pose(39.856, 82.450, Math.toRadians(180));
    private final Pose endPose = new Pose(17.545, 83.029, Math.toRadians(180));

    // --- 路径定义 ---
    private PathChain toShoot, toIntakePath, toEnd;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        mechanism = new MechanismController(robot);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        mechanism.targetRPM = 1800.0; // 起始预热

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            mechanism.update();
            autonomousPathUpdate();

            telemetry.addData("State", pathState);
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();
        }
    }

    public void buildPaths() {
        // 1. 启动 -> 射击点 (直线)
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // 2. 射击点 -> 吸球起点 (贝塞尔曲线)
        toIntakePath = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPose, intakeStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeStartPose.getHeading())
                .build();

        // 3. 吸球起点 -> 终点 (直线)
        toEnd = follower.pathBuilder()
                .addPath(new BezierLine(intakeStartPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private boolean hasReached(Pose target) {
        return Math.hypot(follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()) < POS_TOLERANCE;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 移动至射击点
                follower.followPath(toShoot);
                pathState = 1;
                break;

            case 1: // 等待到达射击点
                if (hasReached(shootPose)) {
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            case 2: // 射击：静止 1.2 秒
                mechanism.requestFire = true;
                if (stateTimer.seconds() > 1.2) {
                    mechanism.requestFire = false;
                    mechanism.targetRPM = 400; // 怠速
                    mechanism.requestGather = true; // 开启吸球

                    follower.followPath(toIntakePath); // 执行曲线路径
                    pathState = 3;
                }
                break;

            case 3: // 移动中，检查到达吸球起点
                if (hasReached(intakeStartPose)) {
                    follower.followPath(toEnd); // 衔接去终点
                    pathState = 4;
                }
                break;

            case 4: // 到达终点
                if (hasReached(endPose)) {
                    mechanism.requestGather = false;
                    mechanism.targetRPM = 0;
                    pathState = -1; // 结束
                }
                break;
        }
    }
}