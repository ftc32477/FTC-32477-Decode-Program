package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_V3_Final_BLUE", group = "Production")
public class TeleOp_V3_Final_BLUE extends TeleOp_V3_Final_Base {

    @Override
    public void runOpMode() {
        // 蓝方专属一键对齐度数
        targetAngleA = 45.0;  // 按住 A 键底盘瞬间定死 45° 近点
        targetAngleB = 30.0;  // 按住 B 键底盘瞬间定死 30° 近点

        // 蓝方初始化车头朝右90°，操作手视线补偿 -90°
        angleOffset = -90.0;

        super.runOpMode();
    }
}