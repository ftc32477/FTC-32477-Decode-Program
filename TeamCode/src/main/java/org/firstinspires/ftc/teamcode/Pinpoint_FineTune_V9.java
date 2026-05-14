package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 32477 V9 精密校准系统 (15度步长微调)
 */
@Autonomous(name = "Pinpoint_FineTune_V9", group = "Calibration")
public class Pinpoint_FineTune_V9 extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private GoBildaPinpointDriver odo;

    // --- 带入你提供的精准初值 ---
    double xOffset = 95.7;
    double yOffset = 38.7;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addLine(">> 精密调试就绪");
        telemetry.addLine(">> 步长：15度 | 驻留：120秒");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 第一阶段：5次精密采样
            for (int i = 1; i <= 5 && opModeIsActive(); i++) {
                odo.setOffsets(xOffset, yOffset);
                odo.update();
                odo.resetPosAndIMU();
                sleep(600);

                // A. 旋转到 15 度 (精密小步长)
                double rotateTime15 = getRuntime();
                while (opModeIsActive() && (getRuntime() - rotateTime15 < 4.0)) {
                    odo.update();
                    double currentAngle = odo.getPosition().getHeading(AngleUnit.DEGREES);
                    double error = 15.0 - currentAngle; // 减小目标角度至15度

                    if (Math.abs(error) < 0.5) break; // 提高停止精度

                    double power = error * 0.022; // 降低增益，动作更柔和
                    power = Math.max(-0.25, Math.min(0.25, power)); // 限制最大功率
                    setDrivePower(0, 0, power);

                    telemetry.addData(">> 精密采样", "%d / 5", i);
                    telemetry.addData("当前角度", "%.2f°", currentAngle);
                    telemetry.update();
                }
                setDrivePower(0, 0, 0);
                sleep(1200); // 延长静止等待时间

                // B. 数据分析
                odo.update();
                Pose2D pos = odo.getPosition();
                double dx = pos.getX(DistanceUnit.MM);
                double dy = pos.getY(DistanceUnit.MM);
                double angleRad = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

                if (Math.abs(angleRad) > 0.03) {
                    xOffset += (dy / angleRad);
                    yOffset -= (dx / angleRad);
                }

                // C. 旋转回 0 度
                double rotateTime0 = getRuntime();
                while (opModeIsActive() && (getRuntime() - rotateTime0 < 4.0)) {
                    odo.update();
                    double currentAngle = odo.getPosition().getHeading(AngleUnit.DEGREES);
                    double error = 0.0 - currentAngle;

                    if (Math.abs(error) < 0.5) break;

                    double power = error * 0.022;
                    power = Math.max(-0.25, Math.min(0.25, power));
                    setDrivePower(0, 0, power);
                    telemetry.update();
                }
                setDrivePower(0, 0, 0);
                sleep(600);
            }

            // 第二阶段：超长锁定展示 (120秒)
            double displayStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - displayStartTime < 120.0)) {
                telemetry.addLine("======== 精密校准完成：结果展示 ========");
                telemetry.addData("【最终 X Offset】", "%.3f mm", xOffset);
                telemetry.addData("【最终 Y Offset】", "%.3f mm", yOffset);
                telemetry.addLine("------------------------------------");
                telemetry.addData("屏幕驻留倒计时", "%.1f 秒", 120.0 - (getRuntime() - displayStartTime));
                telemetry.addLine(">> 记录后请修改代码初值或填入主程序");
                telemetry.update();
                setDrivePower(0, 0, 0);
                idle();
            }
        }
    }

    private void setDrivePower(double drive, double strafe, double turn) {
        double pLF = drive + strafe + turn;
        double pRF = drive - strafe - turn;
        double pLB = drive - strafe + turn;
        double pRB = drive + strafe - turn;
        double max = Math.max(Math.abs(pLF), Math.max(Math.abs(pRF), Math.max(Math.abs(pLB), Math.abs(pRB))));
        if (max > 1.0) { pLF /= max; pRF /= max; pLB /= max; pRB /= max; }
        lf.setPower(pLF); rf.setPower(pRF); lb.setPower(pLB); rb.setPower(pRB);
    }
}