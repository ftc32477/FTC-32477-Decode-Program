package org.firstinspires.ftc.teamcode;

public class MechanismController {
    private RobotHardwareV3 robot;

    // 控制参数
    public double targetRPM = 0;
    public boolean requestFire = false;
    public boolean requestGather = false;

    public MechanismController(RobotHardwareV3 robot) {
        this.robot = robot;
    }

    public void update() {
        // 1. 飞轮控制 (PIDF 闭环)
        if (targetRPM > 500) {
            double ticksPerSec = (targetRPM / 60.0) * robot.SHOOTER_TICKS_PER_REV;
            robot.s1.setVelocity(ticksPerSec);
            robot.s2.setVelocity(ticksPerSec);
        } else {
            robot.s1.setVelocity(0);
            robot.s2.setVelocity(0);
        }

        // 2. 机构动作 (无论底盘状态如何，这里始终执行)
        if (requestFire) {
            // 射击状态
            robot.aservo1.setPosition(0.0);
            robot.aservo2.setPosition(0.0);
            robot.intake.setPower(0.9);
            robot.load.setPower(0.9);
        } else if (requestGather) {
            // 吸球状态
            robot.aservo1.setPosition(0.4);
            robot.aservo2.setPosition(0.4);
            robot.intake.setPower(0.9);
            robot.load.setPower(-0.2); // 阻尼
        } else {
            // 待机
            robot.intake.setPower(0.0);
            robot.load.setPower(0.0);
        }
    }
}