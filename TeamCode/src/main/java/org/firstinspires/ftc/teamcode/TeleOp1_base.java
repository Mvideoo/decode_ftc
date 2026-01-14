package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "анигиляторная пуфка")


public class TeleOp1_base extends LinearOpMode {
    DcMotor LeftRearDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightRearDrive = null;
    DcMotor RightFrontDrive = null;
    DcMotor armMotor = null;
    Servo Zahvat, ArmServo2, armRotServo;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0;
        usbFacingDirectionPosition = 2;
        updateOrientation();

        LeftRearDrive = hardwareMap.dcMotor.get("leftBack");
        RightRearDrive = hardwareMap.dcMotor.get("rightBack");
        LeftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        RightFrontDrive = hardwareMap.dcMotor.get("rightFront");


        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotServo = hardwareMap.servo.get("armRot");
        Zahvat = hardwareMap.servo.get("zahvat");

        ArmServo2 = hardwareMap.servo.get("ArmServo2");


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        ArmServo2.setDirection(Servo.Direction.REVERSE);


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);


        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double lf = (gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.left_trigger * 0.78) - (gamepad1.right_trigger * 0.78)) * 0.8;
            double lb = (gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.left_trigger * 0.78) - (gamepad1.right_trigger * 0.78)) * 0.8;
            double rf = (gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.left_trigger * 0.78) + (gamepad1.right_trigger * 0.78)) * 0.8;
            double rb = (gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.left_trigger * 0.78) + (gamepad1.right_trigger * 0.78)) * 0.8;
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
                lf = -0.2;
                lb = 0.2;
                rf = 0.2;
                rb = -0.2;
            } else if (gamepad1.left_bumper) {
                lf = 0.2;
                lb = -0.2;
                rf = -0.2;
                rb = 0.2;
            }
            if (gamepad1.y) {
                lf = 0;
                lb = 0;
                rf = 0;
                rb = 0;
            }

            if (gamepad2.b) {
                if (ArmServo2.getPosition() < 0.65) {
                    ArmServo2.setPosition(ArmServo2.getPosition() + 0.01);
                    sleep(3);
                }
            }

            if (gamepad2.a) {
                if (ArmServo2.getPosition() > 0.26) {
                    ArmServo2.setPosition(ArmServo2.getPosition() - 0.01);
                    sleep(3);
                }
            }


            if (gamepad2.right_bumper) {
                Zahvat.setPosition(0.44);
                sleep(100);
                armPosition = 7200;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                ArmServo2.setPosition(0.62);
                sleep(40);
                armRotServo.setPosition(0.5);


            } else if (gamepad2.left_bumper) {
                armPosition = 0;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                ArmServo2.setPosition(0.26);
                sleep(20);
                Zahvat.setPosition(0.1);
                armRotServo.setPosition(0.0);

            }
            if (gamepad2.right_stick_y > 0) {
                if (armRotServo.getPosition() < 1) {
                    armRotServo.setPosition(armRotServo.getPosition() + 0.01);
                    sleep(5);
                }

            }
            if (gamepad2.right_stick_y < 0) {
                if (armRotServo.getPosition() > 0) {
                    armRotServo.setPosition(armRotServo.getPosition() - 0.01);
                    sleep(5);
                }
            }


            if (gamepad2.x) {
                if (Zahvat.getPosition() > 0.1) {
                    Zahvat.setPosition(0.1);
                    sleep(200);

                }
            } else if (gamepad2.y) {
                if (Zahvat.getPosition() < 0.44) {
                    Zahvat.setPosition(0.44);
                    sleep(150);
                    ArmServo2.setPosition(0.32);
                    armRotServo.setPosition(0.5);

                }
            }


            if (gamepad2.dpad_down) {
                armPosition = 0;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);

            } else if (gamepad2.dpad_up) {
                armPosition = 7230;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }
            if (gamepad2.left_trigger != 0) {
                lf = (gamepad2.left_stick_y - gamepad2.left_stick_x) * 0.4;
                lb = (gamepad2.left_stick_y + gamepad2.left_stick_x) * 0.4;
                rf = (gamepad2.left_stick_y + gamepad2.left_stick_x) * 0.4;
                rb = (gamepad2.left_stick_y - gamepad2.left_stick_x) * 0.4;
            }

            if (gamepad2.right_trigger != 0) {
                armPosition = 5600;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.7);

            }
            telemetry.addData("", ArmServo2.getPosition());
            telemetry.addData("", armRotServo.getPosition());

            if (orientationIsValid) {
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            } else {
                telemetry.addData("Error", "Selected orientation on robot is invalid");
            }

            telemetry.update();


            armMotor.setTargetPosition((int) armPosition);
            ((DcMotorEx) armMotor).setVelocity(40000);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            armMotor.setPower(0);
            LeftFrontDrive.setPower(lf * 0.8);
            LeftRearDrive.setPower(lb * 0.8);
            RightRearDrive.setPower(rb * 0.8);
            RightFrontDrive.setPower(rf * 0.8);

        }
    }
}




