package org.firstinspires.ftc.teamcode.v2.modules;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigWheelBase.LB_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigWheelBase.LF_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigWheelBase.RB_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigWheelBase.RF_CLOCKWISE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class WheelBase{
    DcMotor LeftFrontDrive, LeftRearDrive, RightFrontDrive, RightRearDrive;
    public WheelBase (HardwareMap hardwareMap){
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        LeftRearDrive = hardwareMap.get(DcMotor.class, "leftBack");
        RightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        RightRearDrive = hardwareMap.get(DcMotor.class, "rightBack");

        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void SetPower(double lf, double lr, double rf, double rr){
        LeftFrontDrive.setPower(lf);
        LeftRearDrive.setPower(lr);
        RightFrontDrive.setPower(rf);
        RightRearDrive.setPower(rr);


    }
    public void SetZero(){
        SetPower(0.0, 0.0, 0.0, 0.0);
    }

    public void clockwiseTest() {
        SetPower(LB_CLOCKWISE, RB_CLOCKWISE, LF_CLOCKWISE, RF_CLOCKWISE);
    }

    public void setAxisPower(double pwX, double pwY, double pwRot) {

        double[] powers = new double[] {
                LB_CLOCKWISE * (pwX - pwY + pwRot),
                RB_CLOCKWISE * (pwX + pwY + pwRot),
                LF_CLOCKWISE * (-pwX - pwY + pwRot),
                RF_CLOCKWISE * (-pwX + pwY + pwRot)
        };

        double maxAbs = 0.0;
        for (double p : powers) {
            maxAbs = Math.max(maxAbs, Math.abs(p));
        }

        double scale = maxAbs > 1.0 ? 1.0 / maxAbs : 1.0;

        SetPower(
                powers[0] * scale,
                powers[1] * scale,
                powers[2] * scale,
                powers[3] * scale
        );
    }


}
