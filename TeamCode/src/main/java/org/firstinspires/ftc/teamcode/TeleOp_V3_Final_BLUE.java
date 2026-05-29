package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_V3_Final_BLUE", group = "Production")
public class TeleOp_V3_Final_BLUE extends TeleOp_V3_Final_Base {

    @Override
    public void runOpMode() {
        // 蓝方专属一键对齐度数
        targetAngleA = 180.0; // 按住 A 键底盘瞬间扭头锁定 180°
        targetAngleB = -45.0; // 按住 B 键底盘斜向倾斜锁定 -45°

        super.runOpMode();
    }
}