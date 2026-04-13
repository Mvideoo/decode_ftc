package org.firstinspires.ftc.teamcode.v2.scenes.teleop.field;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v2.components.AngleSponsor;
import org.firstinspires.ftc.teamcode.v2.components.AprilTagRuler;
import org.firstinspires.ftc.teamcode.v2.components.LookAt;
import org.firstinspires.ftc.teamcode.v2.components.ShooterController;
import org.firstinspires.ftc.teamcode.v2.components.util.Alliance;
import org.firstinspires.ftc.teamcode.v2.modules.CameraOnShooter;
import org.firstinspires.ftc.teamcode.v2.modules.Shooter;
import org.firstinspires.ftc.teamcode.v2.modules.WheelBase;
import org.firstinspires.ftc.teamcode.v2.modules.imu;
import org.firstinspires.ftc.teamcode.v2.systems.ShootingSystem;
import org.firstinspires.ftc.teamcode.v2.systems.Taburetka;

@TeleOp(name = "teleop_decode_RED", group = "TeleOp")
public class teleop_decode_RED extends LinearOpMode {

    @Override
    public void runOpMode() {

        // --- Альянс ---
        Alliance alliance = Alliance.RED;

        // --- Модули (железо) ---
        imu imuModule = new imu(hardwareMap);
        WheelBase wheelBase = new WheelBase(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        CameraOnShooter camera = new CameraOnShooter();
        camera.init(hardwareMap, "shooterCam", null, false);

        // --- Компоненты ---
        AngleSponsor angleSponsor = new AngleSponsor(imuModule);
        AprilTagRuler aprilTagRuler = new AprilTagRuler(camera, alliance);
        aprilTagRuler.open(false);

        ShooterController shooterController = new ShooterController(shooter);
        LookAt lookAt = new LookAt(alliance);

        // --- Системы ---
        Taburetka hogRider = new Taburetka(wheelBase, aprilTagRuler, angleSponsor);
        hogRider.calcAprilTagPos = true;
        hogRider.updateByImu = false;

        ShootingSystem shootingSystem = new ShootingSystem(shooterController, lookAt);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Управление движением ---
            double pwX = -gamepad1.left_stick_x;
            double pwY = -gamepad1.left_stick_y;
            double pwR = gamepad1.right_stick_x;
            hogRider.setAxisPower(pwX, pwY, pwR);

            // --- Стрельба ---
            if (gamepad1.right_trigger > 0.5) {
                shootingSystem.goShoot(hogRider.absX, hogRider.absY, true);
            }

            // --- Тики ---
            aprilTagRuler.tick();
            angleSponsor.tick();
            hogRider.tick();
            shootingSystem.tick();

            telemetry.addData("absX", hogRider.absX);
            telemetry.addData("absY", hogRider.absY);
            telemetry.addData("angle", hogRider.angle);
            telemetry.update();
        }

        aprilTagRuler.close();
    }
}