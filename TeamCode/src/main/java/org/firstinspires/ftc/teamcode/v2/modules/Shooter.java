package org.firstinspires.ftc.teamcode.v2.modules;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooter.kSHT;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooter.kSHT_L;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooter.kSHT_R;

public class Shooter {

    private final DcMotor SHT;
    private final CRServo ShtR;
    private final CRServo ShtL;

    DcMotor LeftFrontDrive;

    public Shooter(HardwareMap hardwareMap) {
        SHT = hardwareMap.get(DcMotor.class, "SHT");
        ShtR = hardwareMap.get(CRServo.class, "ShtR");
        ShtL = hardwareMap.get(CRServo.class, "ShtL");

        SHT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SHT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setShtPower(double pw) {
        SHT.setPower(pw * kSHT);
    }

    public void setFeederPower(double pw) {
        ShtR.setPower(pw * kSHT_R);
        ShtL.setPower(pw * kSHT_L);
    }

        public int getShtTicks() {
        return SHT.getCurrentPosition();
    }

}