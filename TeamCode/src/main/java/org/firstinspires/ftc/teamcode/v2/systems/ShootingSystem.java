package org.firstinspires.ftc.teamcode.v2.systems;

import org.firstinspires.ftc.teamcode.v2.components.LookAt;
import org.firstinspires.ftc.teamcode.v2.components.ShooterController;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooterController;

import java.util.function.Consumer;

public class ShootingSystem {

    private final ShooterController shtControl;
    private final LookAt lookAt;

    public double shooterPreSpeed = 6.0;
    public boolean isTargetAngleReady = false;
    public boolean startShoot = false;
    public boolean positionFixed = false;
    public double absX = 0.0;
    public double absY = 0.0;
    public boolean isNear = true;

    public Consumer<Double> setTargetAngleHandler = angle -> {};

    public ShootingSystem(ShooterController shtControl, LookAt lookAt) {
        this.shtControl = shtControl;
        this.lookAt = lookAt;
    }

    public void tick() {
        if (startShoot && !positionFixed) {
            shtControl.stop = false;
            shtControl.control = true;
            shtControl.targetW = shooterPreSpeed;

            LookAt.Result calcs = lookAt.calcs(absX, absY, shooterPreSpeed, isNear);
            isTargetAngleReady = false;
            setTargetAngleHandler.accept(calcs.rotRobot);

            positionFixed = true;
        }

        if (startShoot && positionFixed && isTargetAngleReady && shtControl.isOnRevolutions()) {
            shoot();
        }

        shtControl.tick();
    }

    private void shoot() {
        startShoot = false;
        positionFixed = false;
        shtControl.stop = true;
    }

    public void goShoot(double absX, double absY, boolean isNear) {
        this.absX = absX;
        this.absY = absY;
        this.isNear = isNear;
        this.shooterPreSpeed = isNear
                ? ConfigShooterController.nearMotorW
                : ConfigShooterController.farMotorW;
        this.startShoot = true;
    }
}