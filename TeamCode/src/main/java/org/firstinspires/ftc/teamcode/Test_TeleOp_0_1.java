package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Test_TeleOp_Final_V4", group = "Development")
public class Test_TeleOp_0_1 extends LinearOpMode {

    private DcMotorEx lf, rf, lb, rb;
    private DcMotorEx intake, load, s1, s2;
    private IMU imu;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // 1. 硬件映射
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        load = hardwareMap.get(DcMotorEx.class, "load");
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");

        // 2. 方向设置 (全部 Forward，逻辑在下面重写)
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        s2.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. 传感器初始化
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        } catch (Exception e) {}

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-120.0, -120.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            // --- 基于你实测反馈的“重映射”逻辑 ---

            // 1. 前后走 (Drive)：你反馈左摇杆左右推是前后走。
            // 既然右推是前移，那么 drive = gamepad1.left_stick_x
            double drive = -gamepad1.left_stick_y;

            // 2. 左右平移 (Strafe)：你反馈右摇杆左右推是平移。
            // 既然左/右推对应左/右平移，直接映射：
            double strafe = gamepad1.left_stick_x;

            // 3. 原地旋转 (Turn)：你反馈左摇杆前/后推是旋转。
            // 既然前推是右旋，后推是左旋，直接映射（注意 Y 轴前推是负值）：
            double turn = gamepad1.right_stick_x;

            /**
             * 横置底盘基础方程：
             * 注意：因为电机横过来了，我们不仅改映射，还需要微调方程中的分量分配
             */
            double flP = -drive + strafe + turn;
            double frP =  drive + strafe + turn;
            double blP = -drive - strafe + turn;
            double brP =  drive - strafe + turn;

            // 归一化
            double max = Math.max(Math.abs(flP), Math.max(Math.abs(frP),
                    Math.max(Math.abs(blP), Math.abs(brP))));
            if (max > 1.0) {
                flP /= max; frP /= max; blP /= max; brP /= max;
            }

            lf.setPower(flP);
            rf.setPower(frP);
            lb.setPower(blP);
            rb.setPower(brP);

            // 子系统部分（保持不变）
            if (gamepad2.a) { intake.setPower(0.8); }
            else if (gamepad2.right_trigger > 0.1) {
                s1.setPower(gamepad2.right_trigger);
                s2.setPower(gamepad2.right_trigger);
            } else {
                intake.setPower(0); s1.setPower(0); s2.setPower(0);
            }

            telemetry.addData("定位(cm)", "X:%.1f, Y:%.1f", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM));
            telemetry.update();
        }
    }
}