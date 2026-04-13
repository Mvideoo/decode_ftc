package org.firstinspires.ftc.teamcode.v2.systems;

import org.firstinspires.ftc.teamcode.v2.components.AngleSponsor;
import org.firstinspires.ftc.teamcode.v2.components.AprilTagRuler;
import org.firstinspires.ftc.teamcode.v2.components.superclasses.PID;
import org.firstinspires.ftc.teamcode.v2.components.util.Calculations;
import org.firstinspires.ftc.teamcode.v2.components.util.funcs;
import org.firstinspires.ftc.teamcode.v2.modules.WheelBase;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigTaburetka;
import java.util.ArrayList;
import java.util.Arrays;

public class Taburetka {

    private final WheelBase wb;
    private final AprilTagRuler aprilTagRuler;
    private final AngleSponsor angleSponsor;

    public double startAbsX = 30.0;
    public double startAbsY = 30.0;

    public double absX = 30.0;
    public double absY = 30.0;
    public double angle = 0.0;

    public boolean updateByImu = false;
    public boolean calcAprilTagPos = false;
    public boolean aprilTagFound = false;

    public double targetX = 30.0;
    public double targetY = 30.0;
    public double targetAngle = 0.0;
    public boolean controlXY = false;
    public boolean controlAngle = false;

    public double pwX = 0.0;
    public double pwY = 0.0;
    public double pwR = 0.0;
    public boolean rotateCoords = true;
    public boolean normPower = true;
    public boolean powerAxis = true;

    public double maxNormPw = 1.0;
    public double pwROverride = 0.0;

    public final PID moveXPID = new PID(ConfigTaburetka.moveXY_kp, ConfigTaburetka.moveXY_ki, ConfigTaburetka.moveXY_kd);
    public final PID moveYPID = new PID(ConfigTaburetka.moveXY_kp, ConfigTaburetka.moveXY_ki, ConfigTaburetka.moveXY_kd);
    public final PID moveRPID = new PID(ConfigTaburetka.moveR_kp, ConfigTaburetka.moveR_ki, ConfigTaburetka.moveR_kd);

    public Taburetka(WheelBase wb, AprilTagRuler aprilTagRuler, AngleSponsor angleSponsor) {
        this.wb = wb;
        this.aprilTagRuler = aprilTagRuler;
        this.angleSponsor = angleSponsor;
    }

    public void setupAngle(double startAngle) {
        angle = startAngle;
        targetAngle = startAngle;
        angleSponsor.setOffset(-startAngle);
    }

    public void setXYPos(double targetX, double targetY) {
        moveXPID.reset();
        moveYPID.reset();
        this.targetX = targetX;
        this.targetY = targetY;
        controlXY = true;
    }

    public void setRotation(double targetAngle) {
        moveRPID.reset();
        this.targetAngle = targetAngle;
        controlAngle = true;
    }

    public void setAxisPower(double pwX, double pwY, double pwR) {
        controlXY = false;
        controlAngle = false;
        this.pwX = pwX;
        this.pwY = pwY;
        this.pwR = pwR;
    }

    private void setAxisPowerTick() {
        double npwX = pwX;
        double npwY = pwY;
        double npwR = pwR;

        if (rotateCoords) {
            Calculations.Coords rotated = Calculations.rotateCoordSystem(npwX, npwY, -angle);
            npwX = rotated.x;
            npwY = rotated.y;
        }

        if (normPower) {
            ArrayList<Double> normed = funcs.normToMax(
                    new ArrayList<>(Arrays.asList(npwX, npwY, npwR)),
                    maxNormPw
            );
            npwX = normed.get(0);
            npwY = normed.get(1);
            npwR = normed.get(2);
        }

        wb.setAxisPower(npwX, npwY, npwR);
    }

    public void tick() {
        if (calcAprilTagPos) {
            absX = aprilTagRuler.absX;
            absY = aprilTagRuler.absY;
            aprilTagFound = aprilTagRuler.absUpdated;
        }

        if (updateByImu) {
            angle = angleSponsor.getCurrAngle();
        } else {
            if (calcAprilTagPos && aprilTagRuler.absUpdated) {
                angle = aprilTagRuler.lastAngle;
            }
        }

        if (controlXY) {
            pwX = moveXPID.tick(targetX - absX);
            pwY = moveYPID.tick(targetY - absY);
        }

        if (controlAngle) {
            pwR = moveRPID.tick(-funcs.constrTo180(angle - targetAngle));
        } else {
            pwR = pwROverride;
        }

        if (powerAxis) {
            setAxisPowerTick();
        }
    }
}