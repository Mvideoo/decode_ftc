package org.firstinspires.ftc.teamcode.v2.components;

import org.firstinspires.ftc.teamcode.v2.modules.imu;
import static org.firstinspires.ftc.teamcode.v2.components.util.funcs.constrTo180;

public class AngleSponsor {

    private final imu imu;
    private double angleOffset = 0.0;

    public AngleSponsor(imu imu) {
        this.imu = imu;
    }

    public double getCurrAngle() {
        return constrTo180(imu.getAngle() + angleOffset);
    }

    public void setOffset(double offset) {
        this.angleOffset = offset;
    }

    public void resetAngle() {
        this.angleOffset = -imu.getAngle();
    }

    public void tick() {
        // пока пусто
    }
}