package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "OpenCV_Blue_4A", group = "Tutorials")
public class blue_opencv_4a extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    private double CrLowerUpdate = 100;
    private double CbLowerUpdate = 150;
    private double CrUpperUpdate = 150;
    private double CbUpperUpdate = 230;

    public static double borderLeftX = 0.0;
    public static double borderRightX = 0.0;
    public static double borderTopY = 0.0;
    public static double borderBottomY = 0.0;

    public double power = -0.25;

    private double lowerruntime = 0;
    private double upperruntime = 0;


    public static Scalar scalarLowerYCrCb = new Scalar(0, 145, 0);
    public static Scalar scalarUpperYCrCb = new Scalar(245, 255.0, 100.0);
    private boolean b = false;

    DcMotor LeftRearDrive, RightRearDrive, LeftFrontDrive;

    @Override
    public void runOpMode() {

        /*LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");

        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

         */


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam norm"), cameraMonitorViewId);

        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("баг");
            }
        });
        /*AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .build();
         */

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            testing(myPipeline);
            telemetry.addData("getRectArea", myPipeline.getRectArea());
            telemetry.addData("getRectHeight", myPipeline.getRectHeight());
            telemetry.addData("x", myPipeline);
            telemetry.addData("getRectArea", myPipeline.getRectMidpointXY());


            if (myPipeline.getRectArea() > 450) {
                double objectWidthPixels = myPipeline.getRectWidth();
                double distanceToObject = calculateDistance(objectWidthPixels);

                double objectHeightPixels = myPipeline.getRectHeight();

                telemetry.addData("Object Width (px)", objectWidthPixels);
                telemetry.addData("Object Height (px)", objectHeightPixels);
                telemetry.addData("Distance to Object (cm)", distanceToObject);
                telemetry.addData("position x", myPipeline.getRectMidpointX());

                if (myPipeline.getRectMidpointX() > 360) {
                    telemetry.addLine("get right");
                    AUTONOMOUS_A1();
                    //Turn_right();
                } else if (myPipeline.getRectMidpointX() > 270 && myPipeline.getRectMidpointX() < 350) {
                    AUTONOMOUS_B1();
                    double distance = (91 * 10) / objectHeightPixels;
                    telemetry.addData("Distance №2 to Object (cm)", distance);
                    //if (distance > 25){
                        //Forward();
                    //}
                    //else if (distance <= 10){
                    //    webcam.closeCameraDevice();
                        //Stop();
                    //}
                    double dist2 = (distance + distanceToObject) / 2;
                    telemetry.addData("", dist2);
                } else if (myPipeline.getRectMidpointX() < 280){
                    telemetry.addLine("get left");
                    AUTONOMOUS_C1();
                    //Turn_left();
                }

             }


            telemetry.update();
        }
    }

    private double calculateDistance(double pixelWidth) {
        return (91 * 30) / (pixelWidth - 4);
    }

    /*void Forward() {
        LeftFrontDrive.setPower(power);
        LeftRearDrive.setPower(power);
        RightRearDrive.setPower(power);
    }

    void Stop() {
        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(0);
        RightRearDrive.setPower(0);
    }

    void Turn_left(){
        LeftFrontDrive.setPower(0.15);
        LeftRearDrive.setPower(-0.15);
        RightRearDrive.setPower(0.15);
    }

    void Turn_right(){
        LeftFrontDrive.setPower(-0.15);
        LeftRearDrive.setPower(0.15);
        RightRearDrive.setPower(-0.15);
    }
    void back(){
        LeftFrontDrive.setPower(0.25);
        LeftRearDrive.setPower(0.25);
        RightRearDrive.setPower(0.25);
    }

     */

    public void testing(ContourPipeline myPipeline) {
        if (lowerruntime + 0.05 < getRuntime()) {
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if (upperruntime + 0.05 < getRuntime()) {
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int) CrLowerUpdate);
        telemetry.addData("lowerCb ", (int) CbLowerUpdate);
        telemetry.addData("UpperCr ", (int) CrUpperUpdate);
        telemetry.addData("UpperCb ", (int) CbUpperUpdate);
    }

    public Double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    public void AUTONOMOUS_A1() {
        if (b) {
            return;
        }
        //webcam.closeCameraDevice();
        b = true;
    }

    public void AUTONOMOUS_B1() {
        if (b) {
            return;
        }
        //webcam.closeCameraDevice();
        b = true;
    }

    public void AUTONOMOUS_C1() {
        if (b) {
            return;
        }
        //webcam.closeCameraDevice();
        b = true;
    }
}