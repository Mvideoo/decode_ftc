
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


@Autonomous(name = "парковка")
public class auto_base extends LinearOpMode {
    DcMotor LeftRearDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightFrontDrive = null;
    DcMotor RightRearDrive = null;
    DcMotor armMotor = null;
    Servo Zahvat, ArmServo2, armRotServo;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    double lf, lb, rf, rb;
    boolean f = false, f2 = false, f3 = false, n = false, h = true, ap = false, first = true, f4 = false, f5 = false, f6 = false;
    boolean f7 = false, f8 = false, f9 = false, f10 = false, f11 = false, g = false, l = false, f12 = false;
    boolean f13 = false, f14 = false,f15 = false,f16 = false,f17 = false,f18 = false,f19 = false,f20 = false;
    int a = 0;


    @Override
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


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmServo2.setDirection(Servo.Direction.REVERSE);


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setTargetPosition(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();


                if (first) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 30) {
                        run();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        first = false;
                        ap = true;
                    }
                }

                if (h) {
                    Stop();
                    up();
                    h = false;
                }


                if (ap) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 33) {
                        right(0.45);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        ap = false;
                        n = true;

                    }
                }
                if (n) {
                    if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) < 27) {
                        left_rot(0.4);
                    }
                    if (Math.abs(orientation.getYaw(AngleUnit.DEGREES)) >= 27) {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f = true;
                        n = false;

                    }
                }
                if (f) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 37) {
                        run();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f = false;
                        f2 = true;

                    }

                }
                if (f2) {
                    sleep(200);
                    Zahvat.setPosition(0.08);
                    sleep(500);
                    armRotServo.setPosition(0.6);
                    f3 = true;
                    f2 = false;
                    a = LeftFrontDrive.getCurrentPosition();
                }
                if (f3) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 42) {
                        back();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f3 = false;
                        f4 = true;

                    }

                }

                if (f4) {
                    down();
                    sleep(1000);
                    f4 = false;
                    f5 = true;
                }
                if (f5) {
                    if (orientation.getYaw(AngleUnit.DEGREES) > -75) {
                        right_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        ArmServo2.setPosition(0.275);
                        armRotServo.setPosition(0.0);

                        f5 = false;
                        g = true;

                    }

                }
                if (g) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 12) {
                        run();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        g = false;
                        f6 = true;
                        sleep(1100);
                        Zahvat.setPosition(0.44);
                        sleep(300);
                    }
                }
                if (f6) {
                    up();
                    sleep(250);
                    f6 = false;
                    f7 = true;

                }
                if (f7) {
                    if (orientation.getYaw(AngleUnit.DEGREES) < 27) {
                        left_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f8 = true;
                        f7 = false;

                    }
                }
                if (f8) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 42) {
                        run();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f8 = false;
                        f9 = true;

                    }
                }
                if (f9) {
                    sleep(350);
                    Zahvat.setPosition(0.08);
                    sleep(500);
                    armRotServo.setPosition(0.6);
                    f10 = true;
                    f9 = false;
                    a = LeftFrontDrive.getCurrentPosition();
                }
                if (f10) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 42) {
                        back();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f10 = false;
                        l = true;

                    }

                }
                if (l) {
                    if (orientation.getYaw(AngleUnit.DEGREES) > -83) {
                        right_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        ArmServo2.setPosition(0.275);
                        armRotServo.setPosition(0.0);
                        down();

                        l = false;
                        f11 = true;

                    }

                }
                if (f11) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 27) {
                        left(0.35);
                    } else {
                        Stop();
                        down();
                        sleep(600);
                        a = LeftFrontDrive.getCurrentPosition();
                        f11 = false;
                        f12 = true;

                    }
                }
                if (f12) {
                    if (orientation.getYaw(AngleUnit.DEGREES) > -83) {
                        right_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        ArmServo2.setPosition(0.275);
                        armRotServo.setPosition(0.0);

                        f12 = false;
                        f13 = true;

                    }

                }
                if (f13) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 5) {
                        back();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f13 = false;
                        f14 = true;
                        sleep(1100);
                        Zahvat.setPosition(0.44);
                        sleep(500);
                    }
                }
                if (f14) {
                    up();
                    sleep(250);
                    f14 = false;
                    f15 = true;

                }
                if (f15) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 27) {
                        right(0.35);
                    } else {
                        Stop();
                        sleep(600);
                        a = LeftFrontDrive.getCurrentPosition();
                        f15 = false;
                        f16 = true;

                    }
                }
                if (f16) {
                    if (orientation.getYaw(AngleUnit.DEGREES) < 23) {
                        left_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f17 = true;
                        f16 = false;

                    }
                }
                if (f17) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 42) {
                        run();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f17 = false;
                        f18 = true;

                    }
                }
                if (f18) {
                    sleep(350);
                    Zahvat.setPosition(0.08);
                    sleep(500);
                    armRotServo.setPosition(0.6);
                    f19 = true;
                    f18 = false;
                    a = LeftFrontDrive.getCurrentPosition();
                }
                if (f19) {
                    if (Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - distance(Math.abs(a))) < 42) {
                        back();
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        f19 = false;
                        f20 = true;

                    }

                }
                if (f20) {
                    if (orientation.getYaw(AngleUnit.DEGREES) > -83) {
                        right_rot(0.4);
                    } else {
                        Stop();
                        a = LeftFrontDrive.getCurrentPosition();
                        ArmServo2.setPosition(0.275);
                        armRotServo.setPosition(0.0);
                        down();

                        f20 = false;
                    }

                }


                telemetry.addData("", LeftFrontDrive.getCurrentPosition());
                telemetry.addData("", a);
                telemetry.addData("", orientation.getYaw(AngleUnit.DEGREES));

                telemetry.addData("", Math.abs(distance(Math.abs(LeftFrontDrive.getCurrentPosition())) - Math.abs(a)));
                telemetry.update();
            }
        }
    }


    double distance(int enc) {
        return ((enc / 537.7) * Math.PI * 10);
    }

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

    private void run() {
        LeftFrontDrive.setPower(-0.4);
        LeftRearDrive.setPower(-0.4);
        RightRearDrive.setPower(-0.4);
        RightFrontDrive.setPower(-0.4);
    }

    private void back() {
        LeftFrontDrive.setPower(0.4);
        LeftRearDrive.setPower(0.4);
        RightRearDrive.setPower(0.4);
        RightFrontDrive.setPower(0.4);
    }

    private void Stop() {
        LeftFrontDrive.setPower(0);
        LeftRearDrive.setPower(0);
        RightRearDrive.setPower(0);
        RightFrontDrive.setPower(0);
    }

    private void right(double power) {

        lf = -power;
        lb = power;
        rf = power;
        rb = -power;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }

    private void left(double power) {
        lf = power;
        lb = -power;
        rf = -power;
        rb = power;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }


    private void left_rot(double power) {
        lf = power;
        lb = power;
        rf = -power;
        rb = -power;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }

    private void right_rot(double power) {
        lf = -power;
        lb = -power;
        rf = power;
        rb = power;
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lb);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rb);
    }

    private void up() {
        Stop();
        Zahvat.setPosition(0.44);
        sleep(100);
        armPosition = 7230;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        ArmServo2.setPosition(0.62);
        sleep(40);
        armRotServo.setPosition(0.465);
        armMotor.setTargetPosition((int) armPosition);
        ((DcMotorEx) armMotor).setVelocity(10000);
    }

    private void down() {
        armPosition = 0;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        ArmServo2.setPosition(0.268);
        sleep(20);
        Zahvat.setPosition(0.1);
        armRotServo.setPosition(0.0);
        armMotor.setTargetPosition((int) armPosition);
        ((DcMotorEx) armMotor).setVelocity(10000);
    }
}
