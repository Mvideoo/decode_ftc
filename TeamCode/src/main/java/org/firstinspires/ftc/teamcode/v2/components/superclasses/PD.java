package org.firstinspires.ftc.teamcode.v2.components.superclasses;

public class PD {

    private final double kp;
    private final double kd;
    private double ErD = 0.0;

    public PD(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
    }

    public void reset() {
        ErD = 0.0;
    }

    public double tick(double Er) {
        double P = Er * kp;
        double D = (Er - ErD) * kd;
        ErD = Er;
        return P + D;
    }
}