package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="shkebede_228_tomato")
public class test12 extends LinearOpMode {
    DcMotor motor;

    public void runOpMode(){
        motor = hardwareMap.dcMotor.get("motor");
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a) {
                motor.setPower(1 + gamepad1.left_trigger * -1 + gamepad1.right_trigger + gamepad1.left_stick_y);
            } else {
                motor.setPower( gamepad1.left_trigger * -1 + gamepad1.right_trigger + gamepad1.left_stick_y);

            }
            
        }
    }

}
