package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "decode_1")


public class test_teleop extends LinearOpMode {
    DcMotor LeftRearDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightRearDrive = null;
    DcMotor RightFrontDrive = null;
    DcMotor Plevaka = null;
    Servo armRotServo;



    public void runOpMode() {
        LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        RightFrontDrive = hardwareMap.dcMotor.get("rightFront");

        Plevaka = hardwareMap.dcMotor.get("Plevaka");


        armRotServo = hardwareMap.servo.get("armRot");


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Plevaka.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double lf = (gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.left_trigger * 0.8) - (gamepad1.right_trigger * 0.8)) * 0.8;
            double lb = (gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.left_trigger * 0.8) - (gamepad1.right_trigger * 0.8)) * 0.8;
            double rf = (gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.left_trigger * 0.8) + (gamepad1.right_trigger * 0.8)) * 0.8;
            double rb = (gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.left_trigger * 0.8) + (gamepad1.right_trigger * 0.8)) * 0.8;
            if (gamepad1.dpad_up) {
                lb = -0.2;
                rb = -0.2;
                rf = -0.2;
                lf = -0.2;
            } else if (gamepad1.dpad_down) {
                lb = 0.2;
                rb = 0.2;
                rf = 0.2;
                lf = 0.2;
            } else if (gamepad1.dpad_left) {
                lf = 0.5;
                lb = -0.5;
                rf = -0.5;
                rb = 0.5;

            } else if (gamepad1.dpad_right) {
                lf = -0.5;
                lb = 0.5;
                rf = 0.5;
                rb = -0.5;
            }

            if (gamepad1.right_bumper) {
                lf = -0.4;
                lb = 0.4;
                rf = 0.4;
                rb = -0.4;
            } else if (gamepad1.left_bumper) {
                lf = 0.4;
                lb = -0.4;
                rf = -0.4;
                rb = 0.4;
            }
            if (gamepad1.y) {
                lf = 0;
                lb = 0;
                rf = 0;
                rb = 0;
            }
            if (gamepad1.a) {
                armRotServo.setPosition(1);
                sleep(300);
                Plevaka.setPower(1);
                sleep(850);
            }
            if (armRotServo.getPosition() >=0.95){
                armRotServo.setPosition(0.35);
            }
            if (armRotServo.getPosition() <= 0.4){
                Plevaka.setPower(0);
            }




            LeftFrontDrive.setPower(lf * 0.85);
            LeftRearDrive.setPower(lb * 0.85);
            RightRearDrive.setPower(rb * 0.85);
            RightFrontDrive.setPower(rf * 0.85);

        }
    }
}




