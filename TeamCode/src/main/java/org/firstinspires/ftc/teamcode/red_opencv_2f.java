package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous(name="OpenCV_Red_2F", group="Tutorials")

public class red_opencv_2f extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 100;
    private double CbLowerUpdate = 150;
    private double CrUpperUpdate = 150;
    private double CbUpperUpdate = 230;

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;


    private boolean b = false;

    //       Range                                   Y    Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(30, 50, 0);
    public static Scalar scalarUpperYCrCb = new Scalar(210, 146, 16);
//    public static Scalar scalarLowerYCrCb = new Scalar(100.0, 200.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 230.0, 120.0);
    DcMotor LeftRearDrive, RightRearDrive, LeftFrontDrive, RightFrontDrive;
    @Override
    public void runOpMode() {
        LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        RightFrontDrive = hardwareMap.dcMotor.get("rightFront");

        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        // Webcam Streaming
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

        // Only if you are using ftcdashboard
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
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 400) {
                    AUTONOMOUS_C();
                } else if (myPipeline.getRectMidpointX() > 200) {
                    AUTONOMOUS_B();
                } else {
                    AUTONOMOUS_A();
                }
            }
        }
    }

    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
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

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }



    public void AUTONOMOUS_A(){
        if(b){
            return;
        }
        webcam.closeCameraDevice();
        telemetry.addLine("Autonomous A");
        b = true;
    }
    public void AUTONOMOUS_B(){
        if(b){
            return;
        }
        webcam.closeCameraDevice();
        telemetry.addLine("Autonomous B");
        b = true;
    }

    public void AUTONOMOUS_C(){
        if(b){
            return;
        }
        webcam.closeCameraDevice();
        telemetry.addLine("Autonomous C");
        b = true;
    }

}

