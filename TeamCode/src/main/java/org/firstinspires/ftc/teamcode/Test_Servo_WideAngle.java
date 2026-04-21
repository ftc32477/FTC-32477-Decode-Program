package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name = "Test_Servo_WideAngle", group = "Test")
public class Test_Servo_WideAngle extends LinearOpMode {

    private ServoImplEx servo1;

    @Override
    public void runOpMode() {
        // 使用 ServoImplEx 以获得更高级的 PWM 控制权限
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");

        // 【关键步骤】设置自定义脉宽范围：300us 到 2700us
        // 这样设置后：setPosition(0) = 300us, setPosition(1) = 2700us
        servo1.setPwmRange(new PwmControl.PwmRange(300, 2700));

        telemetry.addData("状态", "初始化完成，范围设定为 300-2700us");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 如果你想测试它最大的物理旋转范围，可以尝试在 0 和 1 之间切换
            if (gamepad1.a) {
                servo1.setPosition(1.0); // 对应 2700us，也就是 >300度
            } else if (gamepad1.b) {
                servo1.setPosition(0.0); // 对应 300us，也就是 0度
            }

            telemetry.addData("当前目标位置", servo1.getPosition());
            telemetry.update();
        }
    }
}