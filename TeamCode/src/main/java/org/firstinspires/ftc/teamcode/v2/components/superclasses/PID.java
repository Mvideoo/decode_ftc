package org.firstinspires.ftc.teamcode.v2.components.superclasses;

public class PID {
    private final double kp;
    private final double kd;
    private final double ki;

    private double ErD = 0.0;
    private double Ir = 0.0;


    public PID(double kp, double kd, double ki) {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
    }

    public void reset() {
        ErD = 0.0;
        Ir = 0.0;
    }

    public double tick(double Er) {
        double P = Er * kp;
        Ir += Er;
        double I = Ir * ki;
        double D = (Er - ErD) * kd;
        ErD = Er;
        return P + I + D;
    }
}
