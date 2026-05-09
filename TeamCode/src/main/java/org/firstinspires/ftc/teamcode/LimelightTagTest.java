package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight_AprilTag_Test", group = "Test")
public class LimelightTagTest extends LinearOpMode {

    private Limelight3A limelight;

    // ========== 物理常数 (需根据实车精准测量) ==========
    final double CAMERA_HEIGHT = 20.0;    // 相机镜头离地高度 (cm)
    final double TARGET_HEIGHT = 120.0;   // AprilTag 中心离地高度 (cm)
    final double MOUNT_ANGLE = 25.0;      // 相机仰角 (度)

    @Override
    public void runOpMode() {
        // 硬件映射
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 初始化设置
        limelight.pipelineSwitch(0); // 确保在网页端 Pipeline 0 设为 AprilTag 模式
        limelight.start();

        telemetry.addLine("Limelight 已启动，正在等待目标...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // 获取 2D 偏移数据
                double tx = result.getTx(); // 水平偏移角 (度)
                double ty = result.getTy(); // 垂直偏移角 (度)

                // 核心：根据 ty 计算距离 (三角函数法)
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(MOUNT_ANGLE + ty));

                // 尝试获取 3D 姿态数据 (如果需要解析 ID)
                Pose3D botpose = result.getBotpose();

                // 打印信息
                telemetry.addData(">> 状态", "已识别目标");
                telemetry.addData("距离 (Distance)", "%.2f cm", distance);
                telemetry.addData("水平偏移 (tx)", "%.2f °", tx);

                // --- 联盟球门判定逻辑 ---
                // 注意：在 SDK 中，通常通过 result.getFiducialResults() 获取具体 ID 列表
                // 这里我们简化处理，显示当前视野内最近的 Tag 信息
                telemetry.addLine("提示: 请在浏览器确认当前 ID 是否为 20(蓝) 或 24(红)");

                // 辅助驾驶反馈
                if (Math.abs(tx) < 1.5) {
                    telemetry.addLine("结论: 【已对准】");
                } else {
                    telemetry.addLine(tx > 0 ? "结论: 【目标在右】" : "结论: 【目标在左】");
                }

            } else {
                telemetry.addData(">> 状态", "未发现有效 Tag");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}