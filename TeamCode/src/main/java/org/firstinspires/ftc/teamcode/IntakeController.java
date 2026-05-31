package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 32477 Intake 核心控制类 - 独立解耦版
 * 专门接管吸球、蓄球以及5秒周期性揉球逻辑
 * 已经移除了所有与飞轮和开火相关的耦合代码
 */
public class IntakeController {

    private RobotHardwareV3 robot;

    // 5秒蓄球揉球定时器
    private ElapsedTime shuffleTimer = new ElapsedTime();

    public String intakeStatus = "IDLE";

    public IntakeController(RobotHardwareV3 hardware) {
        this.robot = hardware;
        shuffleTimer.reset();
    }

    /**
     * ✨【新增】重载构造函数：专门无缝兼容自动程序硬件类
     */
    public IntakeController(RobotHardwareV3_Auto hardware) {
        // 1. 创建一个干净的 V3 外壳，完美隔离底盘动力轮，防止控制权抢夺
        this.robot = new RobotHardwareV3();

        // 2. 将自动程序里在 hwMap 中绑定好的拾取机构指针无缝转移过来
        this.robot.intake = hardware.intake;
        this.robot.load = hardware.load;

        // 3. 正常重置内部时钟
        shuffleTimer.reset();
    }

    /**
     * 激活常态吸球与定时揉球逻辑（模式一：吸球模式）
     */
    public void runIntakeMode() {
        if (robot == null || robot.intake == null || robot.load == null) return;

        // Intake 常开吸球功率
        robot.intake.setPower(0.9);

        // 5秒揉球循环状态机：前 4.5秒 反转蓄球，后 0.5秒 正转把末端球向里微送
        double cycleTime = shuffleTimer.seconds();
        if (cycleTime >= 5.0) {
            shuffleTimer.reset();
            cycleTime = 0.0;
        }

        if (cycleTime < 4.5) {
            robot.load.setPower(-0.9); // 阻尼反转蓄球
            intakeStatus = "📥 INTAKE [蓄球反转维持中]";
        } else {
            robot.load.setPower(0.4);  // 正转半秒，轻柔把球向里揉
            intakeStatus = "🔄 INTAKE [5s时钟：轻揉推球入内]";
        }
    }

    /**
     * 停止或进入低功耗锁球状态（配合发射模式常态）
     * @param lockWithLowPower 是否开启 0.2 功率锁球
     */
    public void stopOrLock(boolean lockWithLowPower) {
        if (robot == null || robot.intake == null || robot.load == null) return;

        if (lockWithLowPower) {
            robot.intake.setPower(0.2);
            robot.load.setPower(0.0);
            intakeStatus = "🔒 LOCK [0.2功率微弱锁球]";
        } else {
            robot.intake.setPower(0.0);
            robot.load.setPower(0.0);
            intakeStatus = "💤 STANDBY [静止释放]";
        }
        // 重置时钟，确保下次切回吸球模式时从头计算5秒
        shuffleTimer.reset();
    }
}