package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotConfig {
    public GoBildaPinpointDriver odo;
    public int odoResetCount = 0; // 记录重启次数

    public void init(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        // 1. 基础配置
        odo.setOffsets(-120.0, -120.0);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // 2. 执行静置校验
        validateOdoHardware();
    }

    private void validateOdoHardware() {
        boolean isStable = false;
        ElapsedTime timer = new ElapsedTime();

        while (!isStable) {
            odo.resetPosAndIMU(); // 初始/重置
            odoResetCount++;

            // 静置观察 1000 毫秒
            timer.reset();
            double startX = odo.getPosition().getX(DistanceUnit.MM);
            double startY = odo.getPosition().getY(DistanceUnit.MM);

            isStable = true; // 假设它是稳定的
            while (timer.milliseconds() < 1000) {
                odo.update();
                Pose2D currentPos = odo.getPosition();

                // 阈值判定：如果 1 秒内位移超过 0.5 毫米，视为异常漂移
                if (Math.abs(currentPos.getX(DistanceUnit.MM) - startX) > 0.5 ||
                        Math.abs(currentPos.getY(DistanceUnit.MM) - startY) > 0.5) {
                    isStable = false;
                    break; // 触发重启逻辑
                }
            }
        }
    }
}
