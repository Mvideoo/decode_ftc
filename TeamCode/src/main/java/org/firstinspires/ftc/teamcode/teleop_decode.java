package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "teleop_decode", group = "Main")
public class teleop_decode extends LinearOpMode {

    // ===== MOTORS =====
    DcMotor LeftFrontDrive, LeftRearDrive, RightFrontDrive, RightRearDrive;
    DcMotor Plevaka;
    Servo armRotServo;

    // ===== IMU =====
    IMU imu;

    // ===== VISION =====
    VisionPortal visionPortal = null;
    AprilTagProcessor aprilTag = null;
    boolean visionEnabled = false;
    boolean yPrev = false;

    // ===== PID (AprilTag only) =====
    PIDController tagRotPID;
    PIDController tagStrafePID;
    double headingOffset = 0;


    @Override
    public void runOpMode() {

        // ===== HARDWARE =====
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        LeftRearDrive = hardwareMap.get(DcMotor.class, "leftBack");
        RightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        RightRearDrive = hardwareMap.get(DcMotor.class, "rightBack");

        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        armRotServo = hardwareMap.servo.get("armRot");
        Plevaka = hardwareMap.get(DcMotor.class, "Plevaka");
        Plevaka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== IMU =====
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // ===== PID =====
        tagRotPID = new PIDController(0.04, 0.0, 0.002);
        tagStrafePID = new PIDController(0.08, 0.0, 0.003);

        waitForStart();
        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
        armRotServo.setPosition(0.04);


        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS) - headingOffset;


            if (gamepad1.y && !yPrev) {
                visionEnabled = !visionEnabled;
                if (visionEnabled) enableVision();
                else disableVision();
            }
            yPrev = gamepad1.y;

            if (gamepad1.x) {
                headingOffset = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);
            }


            if (visionEnabled && aprilTag != null) {
                centerByAprilTag();
                continue;
            }
            // ===== INPUT =====
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = -gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rot) < 0.05) rot = 0;

            // ===== FIELD CENTRIC =====
            double cosA = Math.cos(heading);
            double sinA = Math.sin(heading);

            double fieldX = x * cosA - y * sinA;
            double fieldY = x * sinA + y * cosA;

            // ===== MECANUM =====
            double lf = fieldY - fieldX + rot;
            double lb = fieldY + fieldX + rot;
            double rf = fieldY + fieldX - rot;
            double rb = fieldY - fieldX - rot;
            telemetry.addData("rot: ", heading);
            telemetry.addData("none rot: ", heading);
            telemetry.update();


            // ===== NORMALIZE =====
            double max = Math.max(
                    Math.abs(lf),
                    Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))
            );

            if (max > 1.0) {
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;
            }

            // ===== DPAD =====
            if (gamepad1.dpad_up) {
                lf = 0.3;
                lb = 0.3;
                rf = 0.3;
                rb = 0.3;
            }
            if (gamepad1.dpad_down) {
                lf = -0.3;
                lb = -0.3;
                rf = -0.3;
                rb = -0.3;
            }
            if (gamepad1.dpad_left) {
                lf = -0.4;
                lb = 0.4;
                rf = 0.4;
                rb = -0.4;
            }
            if (gamepad1.dpad_right) {
                lf = 0.4;
                lb = -0.4;
                rf = -0.4;
                rb = 0.4;
            }

            // ===== APPLY =====
            LeftFrontDrive.setPower(lf * 0.85);
            LeftRearDrive.setPower(lb * 0.85);
            RightFrontDrive.setPower(rf * 0.85);
            RightRearDrive.setPower(rb * 0.85);

            // ===== LAUNCH =====
            if (gamepad1.a){
                Plevaka.setPower(0.8);
                sleep(4000);

                armRotServo.setPosition(0.2);
                sleep(2000);
                Plevaka.setPower(0);
                armRotServo.setPosition(0.04);

            }
            Plevaka.setPower(gamepad1.right_trigger);
        }

        disableVision();
    }

    // ===== APRILTAG =====
    void centerByAprilTag() {
        List<AprilTagDetection> tags = aprilTag.getDetections();
        if (tags.isEmpty()) {
            stopDrive();
            return;
        }

        AprilTagDetection tag = tags.get(0);

        double rot = tagRotPID.update(tag.ftcPose.bearing);
        double strafe = tagStrafePID.update(-tag.ftcPose.x);

        LeftFrontDrive.setPower((strafe + rot) * 0.8);
        LeftRearDrive.setPower((-strafe + rot) * 0.8);
        RightFrontDrive.setPower((-strafe - rot) * 0.8);
        RightRearDrive.setPower((strafe - rot) * 0.8);
    }

    // ===== VISION =====
    void enableVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    void disableVision() {
        if (visionPortal != null) visionPortal.close();
        visionPortal = null;
        aprilTag = null;
    }

    void stopDrive() {
        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightRearDrive.setPower(0);
    }
}
