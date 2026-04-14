package org.firstinspires.ftc.teamcode.v2.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class imu {
    IMU imu;
    double headingOffset = 0.0;


    public imu(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                // параметры гироскопа обязательно поменять
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS) - headingOffset;

    }

    public void resetAngle() {
        headingOffset = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }
}
