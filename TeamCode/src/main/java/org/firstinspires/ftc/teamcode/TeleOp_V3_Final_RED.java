package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_V3_Final_RED", group = "Production")
public class TeleOp_V3_Final_RED extends TeleOp_V3_Final_Base {

    @Override
    public void runOpMode() {
        // 红方专属一键对齐度数
        targetAngleA = -45.0;// 按住 A 键底盘瞬间定死 -45° 近点
        targetAngleB = -30.0;// 按住 B 键底盘瞬间定死 -30° 远点

        super.runOpMode();
    }
}