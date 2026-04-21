/*这个是编码器的设备定义和初始化程序*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotConfig {
    public GoBildaPinpointDriver odo;
    public int odoResetCount = 0;

    public void init(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        // 基础物理参数设置 (单位: mm)
        odo.setOffsets(-120.0, -120.0);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        validateOdoHardware();
    }

    private void validateOdoHardware() {
        boolean isStable = false;
        ElapsedTime timer = new ElapsedTime();

        while (!isStable) {
            odo.resetPosAndIMU();
            odoResetCount++;

            timer.reset();
            isStable = true;

            while (timer.milliseconds() < 800) {
                odo.update();
                Pose2D currentPos = odo.getPosition();

                // 稳定性判定：若波动超过 0.1 cm (即 1mm) 则重启
                if (Math.abs(currentPos.getX(DistanceUnit.CM)) > 0.1 ||
                        Math.abs(currentPos.getY(DistanceUnit.CM)) > 0.1) {
                    isStable = false;
                    break;
                }
            }
        }
    }
}