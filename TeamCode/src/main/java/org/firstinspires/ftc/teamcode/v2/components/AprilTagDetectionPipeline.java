package org.firstinspires.ftc.teamcode.v2.components;

import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigAprilTags;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooterCam;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.*;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {

    private final long detectorPtr;
    private final boolean isTest;

    public ArrayList<AprilTagDetection> detections = new ArrayList<>();

    public AprilTagDetectionPipeline(long detectorPtr, boolean isTest) {
        this.detectorPtr = detectorPtr;
        this.isTest = isTest;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input == null) return null;

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                detectorPtr,
                input,
                ConfigAprilTags.aprilTagSize,
                ConfigShooterCam.fx,
                ConfigShooterCam.fy,
                ConfigShooterCam.cx,
                ConfigShooterCam.cy
        );

        if (isTest) {
            for (AprilTagDetection d : detections) {
                Imgproc.rectangle(
                        input,
                        new Point(d.corners[2].x, d.corners[2].y),
                        new Point(d.corners[0].x, d.corners[0].y),
                        new Scalar(255, 0, 200),
                        3
                );

                Imgproc.putText(
                        input,
                        String.valueOf(d.id),
                        new Point(d.corners[2].x, d.corners[2].y - 30),
                        Imgproc.FONT_HERSHEY_PLAIN,
                        2,
                        new Scalar(255, 0, 200),
                        2
                );
            }
        }

        return input;
    }
}