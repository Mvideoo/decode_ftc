package org.firstinspires.ftc.teamcode.v2.components;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.v2.components.util.Alliance;
import org.firstinspires.ftc.teamcode.v2.components.util.Calculations;
import org.firstinspires.ftc.teamcode.v2.components.util.funcs;
import org.firstinspires.ftc.teamcode.v2.modules.CameraOnShooter;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooterCam;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;

public class AprilTagRuler {

    public enum Motif {
        PPG, PGP, GPP, NULL
    }

    private final CameraOnShooter camSht;
    private final Alliance alliance;

    private final long detectorPtr;
    private final AprilTagDetectionPipeline pipe;
    private final AprilTagDetectionPipeline pipeTest;
    private boolean isTest = false;

    public Motif motif = Motif.NULL;
    public boolean readMotif = true;

    public double absX = 0.0;
    public double absY = 0.0;
    public double lastAngle = 0.0;
    public boolean absUpdated = false;

    public double[] lastRelCoords = new double[]{0.0, 0.0, 0.0};

    private double centerKp = 0.03;
    private double centerKd = 0.008;
    private double centerPrevError = 0.0;

    public double centerThresholdDeg = 2.0;

    private double centerPwR = 0.0;
    private boolean isCentered = false;
    private boolean tagVisible = false;

    public double getCenterPwR() { return centerPwR; }
    public boolean isCentered() { return isCentered; }
    public boolean isTagVisible() { return tagVisible; }

    public AprilTagRuler(CameraOnShooter camSht, Alliance alliance) {
        this.camSht = camSht;
        this.alliance = alliance;

        detectorPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3f, 3
        );
        pipe = new AprilTagDetectionPipeline(detectorPtr, false);
        pipeTest = new AprilTagDetectionPipeline(detectorPtr, true);
    }

    public void open(boolean isStreamToDash) {
        camSht.openWithPipeline(pipe, isStreamToDash);
        isTest = false;
    }

    public void openTest() {
        camSht.openWithPipeline(pipeTest, true);
        isTest = true;
    }

    public void close() {
        absUpdated = false;
        isCentered = false;
        tagVisible = false;
        centerPwR = 0.0;
        centerPrevError = 0.0;
        pipe.detections = new ArrayList<>();
        camSht.close();
    }

    public void tick() {
        if (!camSht.isOpened()) return;

        boolean updatedTemp = false;
        tagVisible = false;

        ArrayList<AprilTagDetection> snapshot = isTest ? pipeTest.detections : pipe.detections;

        for (AprilTagDetection d : snapshot) {
            if (d.id >= 21 && d.id <= 23 && readMotif) {
                switch (d.id) {
                    case 21: motif = Motif.PPG; break;
                    case 22: motif = Motif.PGP; break;
                    case 23: motif = Motif.GPP; break;
                }
            } else {
                updatedTemp = true;
                tagVisible = true;

                if (d.id == 20) {
                    Calculations.Coords relCoords = Calculations.rotateCoordSystem(
                            d.pose.x, d.pose.y,
                            -d.pose.R.get(0, 0) / PI * 180.0 - 45.0
                    );
                    lastRelCoords = new double[]{d.pose.x, d.pose.y, d.pose.R.get(0, 0)};
                    absX = 330 - relCoords.x - ConfigShooterCam.offsetX;
                    absY = 330 - relCoords.y - ConfigShooterCam.offsetY;

                    if (alliance == Alliance.BLUE) {
                        lastAngle = funcs.constrTo180(d.pose.R.get(0, 0) / PI * 180.0 + 45.0);
                    } else {
                        lastAngle = funcs.constrTo180(d.pose.R.get(0, 0) / PI * 180.0 - 135.0);
                    }

                } else if (d.id == 24) {
                    Calculations.Coords relCoords = Calculations.rotateCoordSystem(
                            d.pose.x, d.pose.y,
                            d.pose.R.get(0, 0) / PI * 180.0 + 45.0
                    );
                    lastRelCoords = new double[]{d.pose.x, d.pose.y, d.pose.R.get(0, 0)};
                    absX = 330 - relCoords.x + ConfigShooterCam.offsetX;
                    absY = 30 + relCoords.y + ConfigShooterCam.offsetY;

                    if (alliance == Alliance.RED) {
                        lastAngle = funcs.constrTo180(d.pose.R.get(0, 0) / PI * 180.0 + 45.0);
                    } else {
                        lastAngle = funcs.constrTo180(d.pose.R.get(0, 0) / PI * 180.0 - 135.0);
                    }
                }

                double error = lastAngle;
                double derivative = error - centerPrevError;
                centerPwR = centerKp * error + centerKd * derivative;
                centerPrevError = error;
                isCentered = abs(error) < centerThresholdDeg;
            }
        }

        if (!tagVisible) {
            centerPwR = 0.0;
            isCentered = false;
            centerPrevError = 0.0;
        }

        absUpdated = updatedTemp;
    }
}