package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// ⚡ 终极清零：彻底删除了无法解析的 Point 导包，只保留核心几何与路径类
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_Near_Auto", group = "Production")
public class BlueNearAuto extends OpMode {

    // ==============================================================================
    // 1. 核心组件与战队标准硬件
    // ==============================================================================
    private Follower follower;
    private RobotHardwareV3 robot = new RobotHardwareV3();
    private ElapsedTime stateTimer = new ElapsedTime();

    // ==============================================================================
    // 2. 战队驱动内核参数（与 TeleOp 完美对齐）
    // ==============================================================================
    private double targetRPM = 0.0;
    private double iCurrentPosition = 0.0;

    private final double BANGBANG_TRIGGER_THRESHOLD = 80.0;
    private final double RPM_TOLERANCE_LOWER = 35.0;
    private final double RPM_TOLERANCE_UPPER = 200.0;
    private final double FIRST_ACCEL_GAP = -50.0;
    private final double VALVE_SETTLE_DELAY_SEC = 0.5;
    private final double LPF_ALPHA = 0.75;

    private boolean isFirstAcceleration = true;
    private boolean hasPassedThreshold = false;
    private boolean isValveTimerReset = true;
    private double filteredError1 = 0.0;
    private double filteredError2 = 0.0;
    private ElapsedTime valveTimer = new ElapsedTime();

    // ==============================================================================
    // 3. 自动状态机与虚拟手柄映射
    // ==============================================================================
    private enum AutoState {
        DRIVE_TO_SHOOT_POS,   // 1. 正在跑第 1 段路（前往打球位，开启飞轮热机）
        WAIT_FOR_FIRST_READY, // 2. 已到打球点，等待飞轮转速首次达标并开闸
        BURST_FIRE_3SEC,      // 3. 黄金放行状态：模拟RT+LT，原地暴力倾泻弹药 3 秒
        DRIVE_AND_INTAKE,     // 4. 启动后续路径，开 intake 边走边吸前往节点 3
        AUTO_FINISHED         // 5. 自动安全结束
    }

    private AutoState currentState = AutoState.DRIVE_TO_SHOOT_POS;

    private boolean virtual_RT = false;
    private boolean virtual_LT = false;
    private boolean virtual_intake_active = false; // 边走边吸总闸

    // ==============================================================================
    // 4. 路径链定义 (来自 Visualizer 导出的图形化结构)
    // ==============================================================================
    private PathChain MainChain;

    @Override
    public void init() {
        // A. 初始化底盘跟随器
        follower = Constants.createFollower(hardwareMap);

        // 设置起点坐标
        Pose startPose = new Pose(20.250, 119.500, Math.toRadians(143));
        follower.setStartingPose(startPose);

        // B. 初始化战队 V3 标准硬件
        robot.init(hardwareMap);
        valveTimer.reset();

        // C. 稳健嵌套从 Visualizer 导出的路径链数据
        MainChain = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.250, 119.500), new Pose(40.000, 101.000)))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .addPath(new BezierLine(new Pose(40.000, 101.000), new Pose(46.651, 82.584)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(46.651, 82.584), new Pose(15.952, 82.574)))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {
        // 比赛一键发动，命令底盘跑整条 MainChain
        follower.followPath(MainChain);
        currentState = AutoState.DRIVE_TO_SHOOT_POS;

        // 【热机准备】：视作按下 D-pad 下键，飞轮蓄能设为 1600 RPM
        targetRPM = 1600.0;
        iCurrentPosition = 0.0;
        virtual_RT = true;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        // 必须高频调用：维持 Pedro 底盘位置更新与轨迹跟踪
        follower.update();

        // 维持飞轮内核高频 Bang-Bang 滤波计算
        runShooterCoreEngine();

        // ===================================================================
        // 自动主控状态机
        // ===================================================================
        switch (currentState) {

            case DRIVE_TO_SHOOT_POS:
                // 当第一段路跑完，进入节点 1 时
                if (follower.getCurrentPathNumber() >= 1 || !follower.isBusy()) {
                    currentState = AutoState.WAIT_FOR_FIRST_READY;
                }
                break;

            case WAIT_FOR_FIRST_READY:
                // 检查转速首次达标锁 + 0.5秒门锁解开
                if (hasPassedThreshold && (valveTimer.seconds() >= VALVE_SETTLE_DELAY_SEC)) {
                    virtual_LT = true; // 开启喂弹闸
                    stateTimer.reset(); // 开始计算 3 秒连续砸弹时间
                    currentState = AutoState.BURST_FIRE_3SEC;
                }
                break;

            case BURST_FIRE_3SEC:
                // 原地喷球 3 秒钟
                if (stateTimer.seconds() >= 3.0) {
                    targetRPM = 0; // 熄火飞轮
                    virtual_RT = false;
                    virtual_LT = false;

                    // 开启边走边吸总闸
                    virtual_intake_active = true;
                    currentState = AutoState.DRIVE_AND_INTAKE;
                }
                break;

            case DRIVE_AND_INTAKE:
                // 检查整条 MainChain 是否已经全部跑完（安全停靠在节点 3）
                if (!follower.isBusy()) {
                    virtual_intake_active = false;
                    currentState = AutoState.AUTO_FINISHED;
                }
                break;

            case AUTO_FINISHED:
                break;
        }

        // ===================================================================
        // 遥测看板
        // ===================================================================
        telemetry.addData("自动状态", currentState);
        telemetry.addData("当前行驶段数", follower.getCurrentPathNumber());
        telemetry.addData("全场实时坐标", follower.getPose().toString());
        telemetry.update();
    }

    /**
     * 📥 【32477 原生驱动内核】
     */
    private void runShooterCoreEngine() {
        double currentTargetSpeed = virtual_RT ? targetRPM : 0;

        if (!virtual_RT) {
            isFirstAcceleration = true;
            hasPassedThreshold = false;
            isValveTimerReset = true;
            filteredError1 = 0.0;
            filteredError2 = 0.0;
        }

        double actualRPM1 = (robot.s1.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;
        double actualRPM2 = (robot.s2.getVelocity() / robot.SHOOTER_TICKS_PER_REV) * 60.0;

        if (currentTargetSpeed > 100) {
            double targetTicksPerSec = (currentTargetSpeed / 60.0) * robot.SHOOTER_TICKS_PER_REV;
            double rawError1 = currentTargetSpeed - actualRPM1;
            double rawError2 = currentTargetSpeed - actualRPM2;

            filteredError1 = (LPF_ALPHA * filteredError1) + ((1.0 - LPF_ALPHA) * rawError1);
            filteredError2 = (LPF_ALPHA * filteredError2) + ((1.0 - LPF_ALPHA) * rawError2);

            if (virtual_RT && isFirstAcceleration) {
                if (actualRPM1 >= (targetRPM - FIRST_ACCEL_GAP) && actualRPM2 >= (targetRPM - FIRST_ACCEL_GAP)) {
                    isFirstAcceleration = false;
                }
            }

            if (virtual_RT && !isFirstAcceleration && (filteredError1 >= BANGBANG_TRIGGER_THRESHOLD)) {
                robot.s1.setPower(1.0);
            } else {
                robot.s1.setVelocity(targetTicksPerSec);
            }

            if (virtual_RT && !isFirstAcceleration && (filteredError2 >= BANGBANG_TRIGGER_THRESHOLD)) {
                robot.s2.setPower(1.0);
            } else {
                robot.s2.setVelocity(targetTicksPerSec);
            }
        } else {
            robot.s1.setVelocity(0);
            robot.s2.setVelocity(0);
            robot.s1.setPower(0);
            robot.s2.setPower(0);
        }

        boolean s1SpeedReady = (targetRPM - actualRPM1 <= RPM_TOLERANCE_LOWER) && (actualRPM1 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean s2SpeedReady = (targetRPM - actualRPM2 <= RPM_TOLERANCE_LOWER) && (actualRPM2 - targetRPM <= RPM_TOLERANCE_UPPER);
        boolean isSpeedNowReady = virtual_RT && (targetRPM > 500) && s1SpeedReady && s2SpeedReady;

        if (virtual_RT && !hasPassedThreshold && isSpeedNowReady) {
            hasPassedThreshold = true;
        }

        boolean isBallPathReadyToRelease = false;

        if (hasPassedThreshold) {
            robot.aservo1.setPosition(0.0);
            robot.aservo2.setPosition(0.0);

            if (isValveTimerReset) {
                valveTimer.reset();
                isValveTimerReset = false;
            }
            if (valveTimer.seconds() >= VALVE_SETTLE_DELAY_SEC) {
                isBallPathReadyToRelease = true;
            }
        } else {
            robot.aservo1.setPosition(0.4);
            robot.aservo2.setPosition(0.4);
            isValveTimerReset = true;
        }

        robot.iservo1.setPosition(iCurrentPosition);
        robot.iservo2.setPosition(1.0 - iCurrentPosition);

        double intakePower = 0.0;
        double loadPower = 0.0;

        if (virtual_LT) {
            if (!hasPassedThreshold) {
                intakePower = 0.9;
                loadPower = 0.0;
            } else if (!isBallPathReadyToRelease) {
                intakePower = 0.0;
                loadPower = 0.0;
            } else if (!isSpeedNowReady) {
                intakePower = 0.0;
                loadPower = 0.0;
            } else {
                intakePower = 0.9;
                loadPower = 0.9;
            }
        } else if (virtual_intake_active) {
            intakePower = 0.9;
            loadPower = -0.3;
        } else {
            intakePower = 0.2;
            loadPower = 0.0;
        }

        robot.intake.setPower(intakePower);
        robot.load.setPower(loadPower);
    }
}