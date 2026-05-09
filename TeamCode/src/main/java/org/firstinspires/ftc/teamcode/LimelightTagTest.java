package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Limelight_Final_Test", group = "Test")
public class LimelightTagTest extends LinearOpMode {

    private Limelight3A limelight;

    // --- 物理常数 (需根据实车精准测量) ---
    final double CAMERA_HEIGHT = 20.0;    // cm
    final double TARGET_HEIGHT = 122.0;   // cm
    final double MOUNT_ANGLE = 25.0;      // 度

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight 3A 最终测试程序就绪");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                // 计算距离
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(MOUNT_ANGLE + ty));

                // 获取识别到的所有 AprilTag 结果
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : fiducials) {
                    // 修正：直接获取 ID
                    int tagId = (int) fr.getFiducialId();

                    // 修正：Pose3D 的旋转角获取方式
                    // 如果 getRotation() 报错，尝试直接访问其 orientation 成员或使用 getOrientation()
                    // 这里的写法适配大多数 FTC SDK 环境下的 Pose3D 结构
                    Pose3D targetPose = fr.getTargetPoseCameraSpace();
                    double yaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    telemetry.addData(">> 发现 Tag ID", tagId);
                    telemetry.addData("距离 (Distance)", "%.2f cm", distance);
                    telemetry.addData("水平偏移角 (tx)", "%.2f °", tx);
                    telemetry.addData("Tag 扭转角 (Yaw)", "%.2f °", yaw);

                    // 宽泛对准判定 (±5.0度)
                    if (Math.abs(tx) <= 5.0) {
                        telemetry.addLine("对准状态: 【准心已入框 (±5°)】");
                    } else {
                        telemetry.addLine(tx > 0 ? "对准状态: 【目标偏右】" : "对准状态: 【目标偏左】");
                    }

                    // 联盟球门判定
                    if (tagId == 20) telemetry.addLine("联盟目标: 蓝方球门");
                    else if (tagId == 24) telemetry.addLine("联盟目标: 红方球门");
                }
            } else {
                telemetry.addData(">> 状态", "未发现有效 Tag");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}