package org.firstinspires.ftc.teamcode.v2.components;

import org.firstinspires.ftc.teamcode.v2.components.superclasses.PID;
import org.firstinspires.ftc.teamcode.v2.modules.Shooter;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooterController;

import static java.lang.Math.abs;

public class ShooterController {

    private final Shooter sht;

    public double targetW = 0.0;
    public boolean stop = true;
    public boolean control = false;

    private double vel = 0.0;
    private long velTs = System.currentTimeMillis();
    private int lastTicks = 0;
    private double U = 0.0;

    private final double TICKSTOREV = 1.0 / 28;

    private final PID pid = new PID(
            ConfigShooterController.kp,
            ConfigShooterController.ki,
            ConfigShooterController.kd
    );

    public ShooterController(Shooter sht) {
        this.sht = sht;
    }

    public boolean isOnRevolutions() {
        return abs(targetW - vel) < ConfigShooterController.revolutionsThr;
    }

    public void tick() {
        if (control) {
            if (stop) {
                sht.setShtPower(0.0);
            } else {
                int ticks = sht.getShtTicks();
                long timing = System.currentTimeMillis();
                double t = (timing - velTs) / 1000.0;
                vel = (ticks - lastTicks) / t * TICKSTOREV;
                velTs = timing;
                lastTicks = ticks;
                U += pid.tick(targetW - vel) * t;
                sht.setShtPower(U);
            }
        }
    }
}