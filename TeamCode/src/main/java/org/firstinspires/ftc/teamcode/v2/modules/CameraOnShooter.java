package org.firstinspires.ftc.teamcode.v2.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

public class CameraOnShooter {

    private OpenCvCamera camera;
    private boolean isOpened = false;

    public void init(HardwareMap hwMap, String camName, OpenCvPipeline pipeline, boolean useMonitorView) {

        WebcamName webcam = hwMap.get(WebcamName.class, camName);

        if (useMonitorView) {
            int id = hwMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, id);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam);
        }

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                isOpened = true;
            }

            @Override
            public void onError(int errorCode) {
                isOpened = false;
            }
        });
    }

    public void startDashboardStream() {
        if (camera != null && isOpened) {
            FtcDashboard.getInstance().startCameraStream(camera, 10.0);
        }
    }

    public void openWithPipeline(OpenCvPipeline pipeline, boolean isStreamToDash) {
        if (camera != null) {
            camera.setPipeline(pipeline);
        }
    }

    public void close() {
        if (camera != null) {
            camera.closeCameraDeviceAsync(() -> {});
        }
        isOpened = false;
    }

    public boolean isOpened() {
        return isOpened;
    }
}