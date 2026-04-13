package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import android.util.Size;

import java.util.List;

@TeleOp(name = "teleop_decode", group = "Main")
public class teleop_decode extends LinearOpMode {

    DcMotor LeftFrontDrive, LeftRearDrive, RightFrontDrive, RightRearDrive;
    DcMotorEx Plevaka;
    CRServo armRotServo1, armRotServo2;
    IMU imu;
    VisionPortal visionPortal = null;
    AprilTagProcessor aprilTag = null;
    boolean aPrev = false, bPrev = false;
    boolean isScoring = false;
    PIDController tagRotPID;
    PIDController tagStrafePID;
    PIDController plevakaVelocityPID;
    double headingOffset = 0;
    double alpha = Math.toRadians(73);
    double H0 = 0.283;
    double correctionFactor = 0.85;

    @Override
    public void runOpMode() {

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

        armRotServo1 = hardwareMap.get(CRServo.class, "secServo");
        armRotServo2 = hardwareMap.get(CRServo.class, "twoServo");
        Plevaka = hardwareMap.get(DcMotorEx.class, "Plevaka");
        Plevaka.setDirection(DcMotorSimple.Direction.REVERSE);
        Plevaka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Plevaka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        tagRotPID = new PIDController(0.04, 0.0003, 0.002);
        tagStrafePID = new PIDController(0.01, 0.0001, 0.001);
        plevakaVelocityPID = new PIDController(0.008, 0.0001, 0.0005);

        enableVision();

        telemetry.addLine("INIT COMPLETE");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("Press A to start centering + scoring");
        telemetry.addLine("Press B to cancel scoring");
        telemetry.update();

        waitForStart();
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
            if (gamepad1.x) {
                headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            if (gamepad1.dpad_right) correctionFactor = Math.min(1.0, correctionFactor + 0.02);
            if (gamepad1.dpad_left) correctionFactor = Math.max(0.5, correctionFactor - 0.02);

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rot = -gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rot) < 0.05) rot = 0;

            double cosA = Math.cos(heading);
            double sinA = Math.sin(heading);
            double fieldX = x * cosA - y * sinA;
            double fieldY = x * sinA + y * cosA;

            double lf = fieldY - fieldX + rot;
            double lb = fieldY + fieldX + rot;
            double rf = fieldY + fieldX - rot;
            double rb = fieldY - fieldX - rot;

            if (gamepad1.dpad_up) {
                lf = lb = rf = rb = 0.3;
            }
            if (gamepad1.dpad_down) {
                lf = lb = rf = rb = -0.3;
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

            double max = Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb))));
            if (max > 1.0) {
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;
            }

            if (!isScoring) {
                LeftFrontDrive.setPower(lf * 0.7);
                LeftRearDrive.setPower(lb * 0.7);
                RightFrontDrive.setPower(rf * 0.7);
                RightRearDrive.setPower(rb * 0.7);
            }


            if (gamepad1.a && !aPrev && !isScoring && aprilTag != null) {
                List<AprilTagDetection> tags = aprilTag.getDetections();
                if (!tags.isEmpty()) {
                    isScoring = true;
                    tagRotPID.reset();
                    tagStrafePID.reset();
                    plevakaVelocityPID.reset();

                    double centeringStartTime = getRuntime();
                    boolean centered = false;
                    int failCount = 0;
                    final double CENTERING_TIMEOUT = 4.0;

                    while (opModeIsActive() && (getRuntime() - centeringStartTime) < CENTERING_TIMEOUT && !centered && !gamepad1.b) {
                        centered = centerByAprilTag();
                        if (!hasValidTag()) {
                            failCount++;
                            if (failCount > 20) break;
                        } else {
                            failCount = 0;
                        }
                        sleep(20);
                    }

                    stopDrive();
                    sleep(200);

                    if (opModeIsActive() && !gamepad1.b) {
                        AprilTagDetection currentTag = getCurrentTag();
                        double distanceCm = (currentTag != null) ? getTagDistance(currentTag) : 60.0;
                        double rawVelocity = calculateVelocity(distanceCm);
                        double targetVelocity = rawVelocity * correctionFactor;

                        telemetry.addLine(">>> LAUNCHING <<<");
                        telemetry.addData("Distance", "%.1f cm", distanceCm);
                        telemetry.addData("Velocity", "%.0f ticks/s", targetVelocity);
                        telemetry.update();

                        double launchStartTime = getRuntime();
                        double elapsed;
                        while (opModeIsActive() && (elapsed = getRuntime() - launchStartTime) < 3.0 && !gamepad1.b) {
                            double currentVelocity = Plevaka.getVelocity();
                            double error = targetVelocity - currentVelocity;
                            double power = plevakaVelocityPID.update(error, getRuntime());
                            power = Math.max(Math.min(power, 1.0), -1.0);
                            Plevaka.setPower(power);
                            sleep(10);
                        }

                        if (opModeIsActive() && !gamepad1.b) {
                            armRotServo1.setPower(-0.9);
                            armRotServo2.setPower(0.9);
                            sleep(2500);
                        }
                        if (opModeIsActive() && !gamepad1.b) {
                            armRotServo1.setPower(0);
                            armRotServo2.setPower(0);
                            sleep(500);
                        }
                        if (opModeIsActive() && !gamepad1.b) {
                            Plevaka.setPower(0);
                            sleep(1000);
                        }
                    } else {
                        telemetry.addLine(">>> SCORING CANCELED <<<");
                        telemetry.update();
                        sleep(800);
                    }

                    stopDrive();
                    Plevaka.setPower(0);
                    armRotServo1.setPower(0);
                    armRotServo2.setPower(0);
                    isScoring = false;
                }
            }
            aPrev = gamepad1.a;

            if (gamepad1.b && !bPrev) {
                if (isScoring) {
                    stopDrive();
                    Plevaka.setPower(0);
                    armRotServo1.setPower(0);
                    armRotServo2.setPower(0);
                    isScoring = false;
                    telemetry.addLine(">>> SCORING CANCELED <<<");
                    telemetry.update();
                    sleep(300);
                }
            }
            bPrev = gamepad1.b;

            if (aprilTag != null) {
                List<AprilTagDetection> tags = aprilTag.getDetections();
                telemetry.addData("Tags", tags.size());
                telemetry.addData("Corr", "%.2f", correctionFactor);
                if (!tags.isEmpty()) {
                    AprilTagDetection tag = tags.get(0);
                    telemetry.addData("Center X", "%.0f", tag.center.x);
                    double angle = computeTagAngle(tag);
                    telemetry.addData("Tag Angle", "%.1f°", Math.toDegrees(angle));
                    double distanceCm = getTagDistance(tag);
                    telemetry.addData("Distance", "%.1f cm", distanceCm);
                } else {
                    telemetry.addData("Distance", "N/A");
                }
            }

            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    boolean centerByAprilTag() {
        List<AprilTagDetection> tags = aprilTag.getDetections();

        if (tags.isEmpty()) {
            stopDrive();
            return false;
        }

        AprilTagDetection tag = tags.get(0);
        double frameCenterX = 160.0;
        double errorX = tag.center.x - frameCenterX;
        double tagAngle = computeTagAngle(tag);
        double errorAngle = normalizeRadians(-tagAngle);

        boolean centeredX = Math.abs(errorX) < 25.0;
        boolean centeredAngle = Math.abs(errorAngle) < Math.toRadians(8.0);

        double timeNow = getRuntime();
        double strafePower = tagStrafePID.update(errorX, timeNow);
        double turnPower = tagRotPID.update(errorAngle, timeNow);

        strafePower = Math.max(Math.min(strafePower, 0.45), -0.45);
        turnPower = Math.max(Math.min(turnPower, 0.35), -0.35);

        double lf = strafePower + turnPower;
        double lb = -strafePower + turnPower;
        double rf = -strafePower - turnPower;
        double rb = strafePower - turnPower;

        double maxPower = Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));
        if (maxPower > 1.0) {
            lf /= maxPower;
            lb /= maxPower;
            rf /= maxPower;
            rb /= maxPower;
        }

        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);

        telemetry.addData("ErrX", "%.0f px", errorX);
        telemetry.addData("ErrAng", "%.1f°", Math.toDegrees(errorAngle));
        telemetry.addData("Centered", "%b/%b", centeredX, centeredAngle);

        return centeredX && centeredAngle;
    }

    AprilTagDetection getCurrentTag() {
        List<AprilTagDetection> tags = aprilTag.getDetections();
        return tags.isEmpty() ? null : tags.get(0);
    }

    double getTagDistance(AprilTagDetection tag) {
        double distanceCm = 60.0;
        if (tag.ftcPose != null &&
                tag.ftcPose.z > 0.1 &&
                tag.ftcPose.z < 5.0) {
            distanceCm = Math.abs(tag.ftcPose.z) * 100.0;
        } else if (tag.corners != null && tag.corners.length >= 4) {
            double tagPhysicalSizeMeters = 0.16;
            double focalLengthPixels = 510.0;
            Point topLeft = tag.corners[0];
            Point topRight = tag.corners[1];
            Point bottomRight = tag.corners[2];
            Point bottomLeft = tag.corners[3];
            double topWidth = Math.hypot(topRight.x - topLeft.x, topRight.y - topLeft.y);
            double bottomWidth = Math.hypot(bottomRight.x - bottomLeft.x, bottomRight.y - bottomLeft.y);
            double widthPixels = (topWidth + bottomWidth) / 2.0;
            if (widthPixels > 8.0) {
                double distanceMeters = (tagPhysicalSizeMeters * focalLengthPixels) / widthPixels;
                distanceCm = distanceMeters * 100.0;
            }
        }
        distanceCm += 20.0;
        return Math.max(25.0, Math.min(200.0, distanceCm));
    }

    double calculateVelocity(double distanceCm) {
        double cameraOffsetCm = 43.0;
        double effectiveDistanceCm = distanceCm + cameraOffsetCm;
        effectiveDistanceCm = Math.max(35.0, Math.min(180.0, effectiveDistanceCm));
        double distance = effectiveDistanceCm / 100.0;
        double H0_m = H0;
        double h = H0_m;
        double V0 = 10.0;
        double tolerance = 1e-6;
        int maxIterations = 1000;
        for (int i = 0; i < maxIterations; i++) {
            double cosA = Math.cos(alpha);
            double sinA = Math.sin(alpha);
            double tanA = Math.tan(alpha);
            double hNew = H0_m + 0.75 * distance * tanA - (3.0 * 9.807 * distance * distance) / (32.0 * V0 * V0 * cosA * cosA);
            double term1 = (3.0 * 9.807) / (32.0 * cosA * cosA) * (H0_m - hNew);
            double term2 = 12.0 * distance * sinA * sinA;
            double underSqrt = term1 + term2;
            if (underSqrt < 0.01) underSqrt = 0.01;
            double V0New = distance * Math.sqrt(underSqrt);
            if (Math.abs(hNew - h) < tolerance && Math.abs(V0New - V0) < tolerance) {
                h = hNew;
                V0 = V0New;
                break;
            }
            h = hNew;
            V0 = V0New;
        }
        if (effectiveDistanceCm < 50.0) {
            double scale = 0.6 + 0.4 * (effectiveDistanceCm - 35.0) / 15.0;
            V0 *= scale;
        }
        double ticksPerRev = 23.0;
        double metersPerRev = 0.19;
        return (V0 / metersPerRev) * ticksPerRev;
    }

    double computeTagAngle(AprilTagDetection tag) {
        if (tag.corners == null || tag.corners.length < 4) return 0.0;
        Point topLeft = tag.corners[0];
        Point topRight = tag.corners[1];
        double dx = topRight.x - topLeft.x;
        double dy = topRight.y - topLeft.y;
        return Math.atan2(dy, dx);
    }

    double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    boolean hasValidTag() {
        List<AprilTagDetection> tags = aprilTag.getDetections();
        if (tags.isEmpty()) return false;
        AprilTagDetection tag = tags.get(0);
        return tag.corners != null && tag.corners.length >= 4;
    }

    void enableVision() {
        AprilTagProcessor.Builder processorBuilder = new AprilTagProcessor.Builder()
                .setLensIntrinsics(255, 255, 160, 120)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);

        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Tag0", 0.165, DistanceUnit.METER)
                .addTag(24, "Tag1", 0.165, DistanceUnit.METER)
                .build();

        processorBuilder.setTagLibrary(tagLibrary);
        aprilTag = processorBuilder.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(320, 240))
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
        sleep(1000);
    }

    void stopDrive() {
        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightRearDrive.setPower(0);
    }

    static class PIDController {
        private final double kp, ki, kd;
        private double prevError = 0;
        private double integral = 0;
        private boolean firstCall = true;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public double update(double error, double currentTime) {
            if (firstCall) {
                prevError = error;
                firstCall = false;
                return kp * error;
            }
            double dt = 0.02;
            if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) {
                integral = 0;
            }
            integral += error * dt;
            double derivative = (error - prevError) / dt;
            prevError = error;
            double output = kp * error + ki * integral + kd * derivative;
            if (Math.abs(output) > 1.0) {
                integral -= error * dt;
            }
            if (Math.abs(integral) > 0.3) {
                integral = Math.signum(integral) * 0.3;
            }
            return output;
        }

        public void reset() {
            integral = 0;
            prevError = 0;
            firstCall = true;
        }
    }
}