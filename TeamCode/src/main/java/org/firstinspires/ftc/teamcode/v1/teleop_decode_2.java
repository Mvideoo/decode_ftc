package org.firstinspires.ftc.teamcode.v1;

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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop_decode_2", group = "Main")
public class teleop_decode_2 extends LinearOpMode {

    // ===== VISION =====
    VisionPortal visionPortal = null;
    AprilTagProcessor aprilTag = null;
    boolean visionEnabled = false;
    boolean yPrev = false;



    @Override
    public void runOpMode() {


        // ===== LAUNCH STATE MACHINE =====
        ElapsedTime launchTimer = new ElapsedTime();

        enum LaunchState {
            IDLE,
            MOTOR_RUNNING,
            SERVO_MOVING,
            RESET
        }

        LaunchState launchState = LaunchState.IDLE;



        while (opModeIsActive()) {

            if (gamepad1.y && !yPrev) {
                visionEnabled = !visionEnabled;
                if (visionEnabled) enableVision();
                else disableVision();
            }
            yPrev = gamepad1.y;


            if (visionEnabled && aprilTag != null) {
                centerByAprilTag();
                continue;
            }

        }

        disableVision();
    }

    // ===== APRILTAG =====
    void centerByAprilTag() {
        List<AprilTagDetection> tags = aprilTag.getDetections();
        if (tags.isEmpty()) {
            return;
        }

        AprilTagDetection tag = tags.get(0);

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

}
