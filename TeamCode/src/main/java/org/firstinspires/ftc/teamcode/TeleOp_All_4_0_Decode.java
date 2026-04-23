package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * TeleOp v4.0 Decode - 横置底盘 & Pinpoint 双 IMU 融合版
 * * 职能划分：
 * - Driver 1 (主驾驶/全能):
 * - 移动(LStick), 旋转(RStick), 模式切换(X/Y), 自动瞄准(A=45°/B=66°)
 * - 航向角重置 (Dpad Up=0°, Left=90°, Down=180°, Right=-90°)
 * - 拾取(RT/LB), 装填(LT)
 * - Driver 2 (副驾驶/武器手):
 * - 发射(RT), 装填反转(LB), 反转500RPM(RB), 精确转向(RStick*0.5), 设定转速(Dpad)
 * - **【新增】发射俯仰角控制 (Y=上抬, A=下压)**
 */
@TeleOp(name = "TeleOp_All_4_0_Decode", group = "TeleOp")
public class TeleOp_All_4_0_Decode extends LinearOpMode {

    // ========== 1. 常数定义 (Constants) ==========
    class Constants {
        // 硬件映射名称
        final String CHASSIS_MOTOR_LF = "lf";
        final String CHASSIS_MOTOR_RF = "rf";
        final String CHASSIS_MOTOR_LB = "lb";
        final String CHASSIS_MOTOR_RB = "rb";
        final String MOTOR_INTAKE = "intake";
        final String MOTOR_LOAD = "load";
        final String MOTOR_SHOOTER_1 = "s1";
        final String MOTOR_SHOOTER_2 = "s2";
        final String SERVO_PITCH = "pitchServo"; // 新增：俯仰角舵机
        final String SENSOR_IMU = "imu";
        final String SENSOR_ODO = "odocomputer";         // 新增：Pinpoint 计算机

        // 操纵杆死区
        final double JOYSTICK_DEADZONE = 0.1;

        // 发射机与舵机参数
        final double SHOOTER_TICKS = 28;
        final double SHOOTER_F = 14, SHOOTER_P = 250, SHOOTER_I = 0, SHOOTER_D = 100;
        final int RPM_LONG = 1675, RPM_SIDE = 1275, RPM_BASE = 1100, RPM_TOP = 1450;
        final int RPM_TOLERANCE = 50, RPM_REVERSE = 500;

        // 舵机角度限制 (0.0 到 1.0 之间，需根据实际机械结构调试)
        final double PITCH_MAX = 0.8;
        final double PITCH_MIN = 0.2;
        final double PITCH_STEP = 0.005; // 按住按键时舵机移动的步长

        // 电机固定功率
        final double POWER_INTAKE = 0.95;
        final double POWER_LOAD = 0.95;

        // 自动转向参数
        final double TURN_POWER = 0.8;
        final double TURN_TARGET_TRIANGLE_TOP = -44.0;
        final double TURN_TARGET_FAR = -62.5;

        // 【双 IMU 融合权重】 (权重之和应为 1.0)
        final double WEIGHT_HUB_IMU = 0.3;     // Control Hub 内部 IMU 权重
        final double WEIGHT_PINPOINT_IMU = 0.7;// Pinpoint 高精度 IMU 权重

        final int RUMBLE_MS = 200;
        final double D2_TURN_COEFFICIENT = 0.5;
        final String VERSION = "v4.0.0 Decode";
    }

    // ========== 2. 底盘驱动系统 (ChassisDrive) ==========
    class ChassisDrive {
        DcMotor fl, fr, bl, br;
        Navigation nav;
        boolean isNonLinear = true;
        boolean isFieldCentric = true;

        ChassisDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, Navigation nav) {
            this.fl = fl; this.fr = fr; this.bl = bl; this.br = br; this.nav = nav;
        }

        void init() {
            // 对齐 Test_Chassis_4_0 测试结果
            fl.setDirection(DcMotor.Direction.FORWARD);
            bl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
            br.setDirection(DcMotor.Direction.REVERSE);

            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        void toggleMode() { isNonLinear = !isNonLinear; }
        void toggleCentric() { isFieldCentric = !isFieldCentric; }

        String getModeStr() {
            return (isFieldCentric ? "地面 Field" : "车辆 Robot") + " / " + (isNonLinear ? "非线性 Curve" : "线性 Linear");
        }

        void update(double drive, double strafe, double turn, boolean isAutoTurn, double autoPower) {
            if (Math.abs(drive) < constants.JOYSTICK_DEADZONE) drive = 0;
            if (Math.abs(strafe) < constants.JOYSTICK_DEADZONE) strafe = 0;

            if (isAutoTurn) {
                turn = autoPower;
            } else {
                if (Math.abs(turn) < constants.JOYSTICK_DEADZONE) turn = 0;
                if (isNonLinear) {
                    drive = Math.copySign(drive * drive, drive);
                    strafe = Math.copySign(strafe * strafe, strafe);
                    turn = Math.copySign(turn * turn, turn);
                }
            }

            // 无头模式坐标转换
            if (isFieldCentric) {
                double botHeading = Math.toRadians(nav.getHeading());
                double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
                strafe = rotX;
                drive = rotY;
            }

            // 核心算法更改：适用 4.0 横置底盘
            double fL = -drive - strafe - turn;
            double fR = -drive + strafe + turn;
            double bL = -drive + strafe - turn;
            double bR = -drive - strafe + turn;

            double max = Math.max(Math.abs(fL), Math.max(Math.abs(fR), Math.max(Math.abs(bL), Math.abs(bR))));
            if (max > 1.0) { fL /= max; fR /= max; bL /= max; bR /= max; }

            fl.setPower(fL); fr.setPower(fR); bl.setPower(bL); br.setPower(bR);
        }
        void stop() { fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0); }
    }

    // ========== 3. 子系统 (Subsystems) ==========
    class Subsystems {
        DcMotor intake, load;
        DcMotorEx s1, s2;
        Servo pitchServo;

        int targetRPM = 0;
        boolean isShooting = false;
        int currentRunningRPM = 0;
        int currentDirection = 0;
        double currentPitch = 0.5; // 舵机初始位置 (居中)

        Subsystems(DcMotor intake, DcMotor load, DcMotorEx s1, DcMotorEx s2, Servo pitchServo) {
            this.intake = intake; this.load = load; this.s1 = s1; this.s2 = s2; this.pitchServo = pitchServo;
        }

        void init() {
            intake.setDirection(DcMotor.Direction.FORWARD);
            load.setDirection(DcMotor.Direction.REVERSE);

            // 新车巧思：两个同向旋转的电机。将它们设置为同向即可。
            s1.setDirection(DcMotor.Direction.FORWARD);
            s2.setDirection(DcMotor.Direction.FORWARD);

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            PIDFCoefficients pidf = new PIDFCoefficients(constants.SHOOTER_P, constants.SHOOTER_I, constants.SHOOTER_D, constants.SHOOTER_F);
            s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 初始化舵机位置
            currentPitch = (constants.PITCH_MAX + constants.PITCH_MIN) / 2.0;
            pitchServo.setPosition(currentPitch);
        }

        void setTargetRPM(int rpm) { this.targetRPM = rpm; }
        void setShooting(boolean active) { this.isShooting = active; }

        void setActualVelocity(int rpm, int direction) {
            this.currentRunningRPM = rpm;
            this.currentDirection = direction;
            double v_magnitude = rpm * constants.SHOOTER_TICKS / 60.0;
            s1.setVelocity(v_magnitude * direction);
            s2.setVelocity(v_magnitude * direction);
        }

        // 调节俯仰角舵机
        void adjustPitch(double delta) {
            currentPitch = Range.clip(currentPitch + delta, constants.PITCH_MIN, constants.PITCH_MAX);
            pitchServo.setPosition(currentPitch);
        }

        double getShooter1RPM() { return (s1.getVelocity() / constants.SHOOTER_TICKS) * 60.0; }
        double getShooter2RPM() { return (s2.getVelocity() / constants.SHOOTER_TICKS) * 60.0; }

        boolean isAtSpeed() {
            if (!isShooting || targetRPM <= 0) return false;
            boolean s1_ready = Math.abs(getShooter1RPM() - targetRPM) < constants.RPM_TOLERANCE;
            boolean s2_ready = Math.abs(getShooter2RPM() - targetRPM) < constants.RPM_TOLERANCE;
            return s1_ready && s2_ready;
        }

        void runIntake(int dir) { intake.setPower(dir * constants.POWER_INTAKE); }
        void runLoad(int dir) { load.setPower(dir * constants.POWER_LOAD); }

        void stopAll() {
            runIntake(0); runLoad(0);
            setShooting(false); setActualVelocity(0, 0);
        }

        String getIntakeStr() { return intake.getPower() > 0 ? "正转" : (intake.getPower() < 0 ? "反转" : "停止"); }
        String getLoadStr() { return load.getPower() > 0 ? "正转" : (load.getPower() < 0 ? "反转" : "停止"); }
        String getShooterStatusStr() {
            if (currentDirection == 1) return "发射";
            if (currentDirection == -1) return "反转";
            return "停止";
        }
    }

    // ========== 4. 导航系统 (Navigation - 双 IMU 融合) ==========
    class Navigation {
        IMU imu;
        GoBildaPinpointDriver odo; // 新增：Pinpoint

        boolean autoTurning = false;
        double currentAutoTurnTarget = 0.0;
        double yawOffset = 0.0;

        double fusedHeading = 0.0; // 融合后的原始偏航角

        Navigation(IMU imu, GoBildaPinpointDriver odo) {
            this.imu = imu;
            this.odo = odo;
        }

        void init() {
            // 1. Control Hub IMU 初始化
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));
            imu.resetYaw();

            // 2. Pinpoint 初始化 (对齐 Test_OdoStable 配置)
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setOffsets(-120.0, -120.0); // 注意：请在实际调车时再次确认新车的真实 Offset
            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            odo.resetPosAndIMU();

            yawOffset = 0.0;
        }

        // 必须在主循环中每次调用！
        void update() {
            odo.update(); // 更新 Pinpoint 数据

            // 获取两个传感器的原始读数
            double hubYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double odoYaw = Math.toDegrees(odo.getHeading()); // Pinpoint 默认返回弧度，需转换为角度

            // 计算角度差 (规避 -180 到 180 的跳变)
            double deltaYaw = normalize(hubYaw - odoYaw);

            // 融合计算：以 Pinpoint 为基准，加上一定权重的 Hub 差值
            fusedHeading = normalize(odoYaw + constants.WEIGHT_HUB_IMU * deltaYaw);
        }

        double getHeading() {
            // 返回融合后并加上软件补偿的最终角度
            return normalize(fusedHeading + yawOffset);
        }

        void resetTo(double targetHeading) {
            yawOffset = normalize(targetHeading - fusedHeading);
            autoTurning = false;
        }

        void startAutoTurn(double targetHeading) {
            currentAutoTurnTarget = targetHeading;
            autoTurning = true;
        }

        double getAutoTurnPower() {
            if (!autoTurning) return 0;
            double error = normalize(getHeading() - currentAutoTurnTarget);
            if (Math.abs(error) < 2.0) {
                autoTurning = false; return 0;
            }
            return Range.clip(error * 0.05, -constants.TURN_POWER, constants.TURN_POWER);
        }

        double normalize(double d) {
            while (d >= 180) d -= 360;
            while (d < -180) d += 360;
            return d;
        }
    }

    // ========== 5. 输入控制 (InputHandler) ==========
    class InputHandler {
        boolean lastY=false, lastX=false, lastOpt=false;
        boolean lastRight=false, lastLeft=false, lastUp=false, lastDown=false;
        boolean lastA_P1=false, lastB_P1=false;
        boolean lastRight_P1=false, lastLeft_P1=false, lastUp_P1=false, lastDown_P1=false;
        boolean d2OverrideActive = false;

        double getDriveY() { return -gamepad1.left_stick_y; }
        double getDriveX() { return gamepad1.left_stick_x; }

        double getTurnComposite() {
            double p2 = gamepad2.right_stick_x;
            double p1 = gamepad1.right_stick_x;
            if (Math.abs(p2) > constants.JOYSTICK_DEADZONE) {
                if (!d2OverrideActive) { gamepad1.rumble(constants.RUMBLE_MS); d2OverrideActive = true; }
                return p2 * constants.D2_TURN_COEFFICIENT;
            } else {
                d2OverrideActive = false; return p1;
            }
        }

        int getIntakeDir() {
            if (gamepad1.right_trigger > 0.1) return 1;
            if (gamepad1.left_bumper) return -1;
            return 0;
        }

        int getLoadDir() {
            if (gamepad1.left_trigger > 0.1) return 1;
            if (gamepad2.left_bumper) return -1;
            return 0;
        }

        boolean isShoot() { return gamepad2.right_trigger > 0.1; }
        boolean isShooterReverse() { return gamepad2.right_bumper; }

        // --- 舵机角度调整 (D2 Y 和 A 按住调节) ---
        boolean isPitchUp() { return gamepad2.y; }
        boolean isPitchDown() { return gamepad2.a; }

        boolean isAimTriangleTopPressed() { boolean c=gamepad1.a; boolean r=c&&!lastA_P1; lastA_P1=c; return r; }
        boolean isAimFarPressed() { boolean c=gamepad1.b; boolean r=c&&!lastB_P1; lastB_P1=c; return r; }
        boolean isModePressed() { boolean c=gamepad1.y; boolean r=c&&!lastY; lastY=c; return r; }
        boolean isCentricPressed() { boolean c=gamepad1.x; boolean r=c&&!lastX; lastX=c; return r; }

        boolean isReset0Pressed() { boolean c=gamepad1.dpad_up; boolean r=c&&!lastUp_P1; lastUp_P1=c; return r; }
        boolean isReset90Pressed() { boolean c=gamepad1.dpad_left; boolean r=c&&!lastLeft_P1; lastLeft_P1=c; return r; }
        boolean isReset180Pressed() { boolean c=gamepad1.dpad_down; boolean r=c&&!lastDown_P1; lastDown_P1=c; return r; }
        boolean isResetNeg90Pressed() { boolean c=gamepad1.dpad_right; boolean r=c&&!lastRight_P1; lastRight_P1=c; return r; }

        int checkRPM(int current) {
            int next = current;
            if (dpad(gamepad2.dpad_right, 1)) next = constants.RPM_LONG;
            else if (dpad(gamepad2.dpad_left, 2)) next = constants.RPM_SIDE;
            else if (dpad(gamepad2.dpad_down, 3)) next = constants.RPM_BASE;
            else if (dpad(gamepad2.dpad_up, 4)) next = constants.RPM_TOP;
            return next;
        }

        boolean dpad(boolean curr, int id) {
            boolean trig = false;
            if (id==1) { trig = curr && !lastRight; lastRight = curr; }
            else if (id==2) { trig = curr && !lastLeft; lastLeft = curr; }
            else if (id==3) { trig = curr && !lastDown; lastDown = curr; }
            else if (id==4) { trig = curr && !lastUp; lastUp = curr; }
            return trig;
        }

        void rumbleDriver2() { gamepad2.rumble(constants.RUMBLE_MS); }
    }

    // ========== 6. 主程序 (Main Loop) ==========
    Constants constants;
    ChassisDrive chassis;
    Subsystems subsys;
    Navigation nav;
    InputHandler input;
    ElapsedTime runtime;
    boolean rpmReadyVibrationTriggered = false;

    @Override
    public void runOpMode() {
        constants = new Constants();
        runtime = new ElapsedTime();

        try {
            DcMotor fl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_LF);
            DcMotor fr = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_RF);
            DcMotor bl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_LB);
            DcMotor br = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_RB);
            DcMotor intake = hardwareMap.get(DcMotor.class, constants.MOTOR_INTAKE);
            DcMotor load = hardwareMap.get(DcMotor.class, constants.MOTOR_LOAD);
            DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, constants.MOTOR_SHOOTER_1);
            DcMotorEx s2 = hardwareMap.get(DcMotorEx.class, constants.MOTOR_SHOOTER_2);
            Servo pitchServo = hardwareMap.get(Servo.class, constants.SERVO_PITCH);

            IMU imu = hardwareMap.get(IMU.class, constants.SENSOR_IMU);
            GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, constants.SENSOR_ODO);

            nav = new Navigation(imu, odo);
            chassis = new ChassisDrive(fl, fr, bl, br, nav);
            subsys = new Subsystems(intake, load, s1, s2, pitchServo);
            input = new InputHandler();

            nav.init();
            chassis.init();
            subsys.init();
        } catch (Exception e) {
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addData("Status", constants.VERSION + " Dual Driver & Dual IMU Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // 务必每次循环更新导航系统 (抓取 Pinpoint 读数并进行 IMU 融合)
            nav.update();

            // --- Driver 1: 功能键与自动瞄准 ---
            if (input.isReset0Pressed()) { nav.resetTo(0.0); gamepad1.rumble(constants.RUMBLE_MS); }
            if (input.isReset90Pressed()) { nav.resetTo(90.0); gamepad1.rumble(constants.RUMBLE_MS); }
            if (input.isReset180Pressed()) { nav.resetTo(180.0); gamepad1.rumble(constants.RUMBLE_MS); }
            if (input.isResetNeg90Pressed()) { nav.resetTo(-90.0); gamepad1.rumble(constants.RUMBLE_MS); }

            if (input.isModePressed()) { chassis.toggleMode(); gamepad1.rumble(200); }
            if (input.isCentricPressed()) { chassis.toggleCentric(); gamepad1.rumble(200); }

            if (input.isAimTriangleTopPressed()) nav.startAutoTurn(constants.TURN_TARGET_TRIANGLE_TOP);
            if (input.isAimFarPressed()) nav.startAutoTurn(constants.TURN_TARGET_FAR);

            // --- Driver 2: 武器控制 ---
            // 1. 转速切换
            int newRPM = input.checkRPM(subsys.targetRPM);
            if (newRPM != subsys.targetRPM) {
                subsys.setTargetRPM(newRPM);
                input.rumbleDriver2();
                rpmReadyVibrationTriggered = false;
            }

            // 2. 发射机启停
            int targetVelocityRPM = 0;
            int direction = 0;
            if (input.isShooterReverse()) {
                targetVelocityRPM = constants.RPM_REVERSE;
                direction = -1;
                subsys.setShooting(false);
            } else if (input.isShoot()) {
                targetVelocityRPM = subsys.targetRPM;
                direction = 1;
                subsys.setShooting(true);
            } else {
                targetVelocityRPM = 0; direction = 0;
                subsys.setShooting(false);
            }
            subsys.setActualVelocity(targetVelocityRPM, direction);

            // 3. 舵机角度调节 (按住 Y/A 平滑调节)
            if (input.isPitchUp()) {
                subsys.adjustPitch(constants.PITCH_STEP);
            } else if (input.isPitchDown()) {
                subsys.adjustPitch(-constants.PITCH_STEP);
            }

            // 4. 武器状态震动反馈
            if (subsys.isShooting && subsys.isAtSpeed()) {
                if (!rpmReadyVibrationTriggered) {
                    gamepad2.rumble(300);
                    rpmReadyVibrationTriggered = true;
                }
            } else {
                rpmReadyVibrationTriggered = false;
            }

            subsys.runIntake(input.getIntakeDir());
            subsys.runLoad(input.getLoadDir());

            // --- 底盘控制 ---
            chassis.update(
                    input.getDriveY(),
                    input.getDriveX(),
                    input.getTurnComposite(),
                    nav.autoTurning,
                    nav.getAutoTurnPower()
            );

            // --- 遥测信息更新 ---
            telemetry.addData("版本", "%s", constants.VERSION);
            telemetry.addLine();

            telemetry.addData("目标转速", "%d RPM / %s", subsys.targetRPM, subsys.isAtSpeed() ? "✔ 已达标" : "✘ 未达标");
            telemetry.addData("S1/S2 实测", "%.0f / %.0f RPM", subsys.getShooter1RPM(), subsys.getShooter2RPM());
            telemetry.addData("出球舵机", "%.2f (范围: %.2f - %.2f)", subsys.currentPitch, constants.PITCH_MIN, constants.PITCH_MAX);
            telemetry.addLine();

            telemetry.addData("驱动模式", "%s", chassis.getModeStr());
            telemetry.addData("融合航向角", "%.1f °", nav.getHeading());
            telemetry.addData("Odo X / Y", "%.1f cm / %.1f cm",
                    nav.odo.getPosition().getX(DistanceUnit.CM),
                    nav.odo.getPosition().getY(DistanceUnit.CM));
            telemetry.addLine();

            telemetry.addData("Driver 1", "A/B=自动瞄准, RT=拾取, LB=反向, LT=装填");
            telemetry.addData("Driver 2", "RT=发射, Dpad=调速, Y/A=调角度(按住)");

            telemetry.update();
        }

        chassis.stop();
        subsys.stopAll();
    }
}