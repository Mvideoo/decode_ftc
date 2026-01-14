
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "decode_парковка")
public class auto_decode extends LinearOpMode {
    DcMotor LeftRearDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightFrontDrive = null;
    DcMotor RightRearDrive = null;

    @Override
    public void runOpMode() {

        LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        RightFrontDrive = hardwareMap.dcMotor.get("rightFront");


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        boolean f = true;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (f){
                LeftFrontDrive.setPower(-0.5);
                LeftRearDrive.setPower(-0.5);
                RightFrontDrive.setPower(-0.5);
                RightRearDrive.setPower(-0.5);
                sleep(1000);
                f = false;

                }

                LeftFrontDrive.setPower(0);
                LeftRearDrive.setPower(0);
                RightFrontDrive.setPower(0);
                RightRearDrive.setPower(0);

            }
        }
    }




}
