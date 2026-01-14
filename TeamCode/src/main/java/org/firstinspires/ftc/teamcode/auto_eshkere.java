
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "auto_last", group = "Concept")
public class auto_eshkere extends LinearOpMode {

    DcMotor LeftRearDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightFrontDrive = null;
    DcMotor RightRearDrive = null;
    DcMotor armMotor = null;
    Servo Zahvat, ArmServo2, armRotServo;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean f = false, f2 = false, f3 = false, f4 = false, f5 = false, f6 = false, f7 = true, f8 = false;
    double lf, lb, rf, rb;
    int a, b, c;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0;
        usbFacingDirectionPosition = 2;
        updateOrientation();

        LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        RightFrontDrive = hardwareMap.dcMotor.get("rightFront");

        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotServo = hardwareMap.servo.get("armRot");
        Zahvat = hardwareMap.servo.get("zahvat");

        ArmServo2 = hardwareMap.servo.get("ArmServo2");


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmServo2.setDirection(Servo.Direction.REVERSE);


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armMotor.setTargetPosition(0);


        initAprilTag();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();
                telemetry.addData("", LeftFrontDrive.getCurrentPosition());
                telemetry.update();
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                Zahvat.setPosition(0.08);
                ArmServo2.setPosition(0.62);
                if (f7) {
                    armPosition = 7200;
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                    ArmServo2.setPosition(0.62);
                    armRotServo.setPosition(0.6);
                    armMotor.setTargetPosition((int) armPosition);
                    ((DcMotorEx) armMotor).setVelocity(40000);

                    f7 = false;
                }
                if (distance(Math.abs(LeftFrontDrive.getCurrentPosition())) < 55) {
                    LeftFrontDrive.setPower(-0.4);
                    RightRearDrive.setPower(-0.4);

                    LeftRearDrive.setPower(-0.4);
                    RightFrontDrive.setPower(-0.4);


                } else {
                    Stop();
                    a = LeftFrontDrive.getCurrentPosition();
                    f = true;
                }
                if (f) {
                    if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 60) {
                        left_rot();
                    }
                    if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) >= 60) {
                        Stop();

                        f2 = true;
                    }


                if (f2) {
                        if (distance(Math.abs(LeftFrontDrive.getCurrentPosition())) < (120 * distance(a))) {
                            LeftFrontDrive.setPower(-0.6);
                            RightRearDrive.setPower(-0.6);
                            LeftRearDrive.setPower(-0.6);
                            RightFrontDrive.setPower(-0.6);


                        } else {
                            armRotServo.setPosition(0.4);
                            Stop();
                            f3 = true;
                            f8 = true;


                            b = LeftFrontDrive.getCurrentPosition();
                        }

                        if (f8) {
                            if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 0) {
                                left_rot();
                            }
                            if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) >= 0) {
                                Stop();
                                f4 = true;
                                f8 = false;

                            }
                        }

                        if (f4) {
                            armPosition = 0;
                            Zahvat.setPosition(0.08);
                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armMotor.setPower(1);
                            ArmServo2.setPosition(0.7);
                            armRotServo.setPosition(0.1);
                            armMotor.setTargetPosition((int) armPosition);
                            ((DcMotorEx) armMotor).setVelocity(40000);
                            Stop();

                        }
                    }
                }


            }
        }
        visionPortal.close();

    }

    double distance(int enc) {
        return (enc / 537.7 * Math.PI * 104);
    }

    int enc(double distance) {
        return (int) ((distance * 537.7) / (Math.PI * 104));
    }

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

    private void run() {
        LeftFrontDrive.setPower(-0.5);
        LeftRearDrive.setPower(-0.5);
        RightRearDrive.setPower(-0.5);
        RightFrontDrive.setPower(-0.5);
    }

    private void back() {
        LeftFrontDrive.setPower(0.5);
        LeftRearDrive.setPower(0.5);
        RightRearDrive.setPower(0.5);
        RightFrontDrive.setPower(0.5);
    }

    private void Stop() {
        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(0);
        RightRearDrive.setPower(0);
        RightFrontDrive.setPower(0);
    }

    private void right() {
        lf = -1;
        lb = 1;
        rf = 1;
        rb = -1;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);

    }

    private void left_rot() {
        lf = 0.2;
        lb = 0.2;
        rf = -0.2;
        rb = -0.2;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }

    private void right_rot() {
        lf = -0.3;
        lb = -0.3;
        rf = 0.3;
        rb = 0.3;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()

                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x * 2.54, detection.ftcPose.y * 2.54, detection.ftcPose.z * 2.54));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range * 2.54, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}
