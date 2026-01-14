package org.firstinspires.ftc.teamcode;

public class PIDController {

    double kP, kI, kD;
    double integral = 0;
    double lastError = 0;
    long lastTime = 0;

    double integralLimit = 1.0;

    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        lastTime = System.nanoTime();
    }

    public double update(double error) {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
