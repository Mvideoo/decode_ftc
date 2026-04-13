package org.firstinspires.ftc.teamcode.v2.components;

import static org.firstinspires.ftc.teamcode.v2.components.util.funcs.constrTo180;

import org.firstinspires.ftc.teamcode.v2.components.util.Alliance;
import org.firstinspires.ftc.teamcode.v2.components.util.Calculations;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigLookAt;

public class LookAt {



    public static class Result {
        public final double rotRobot;
        public final double shooterAngle;

        public Result(double rotRobot, double shooterAngle) {
            this.rotRobot = rotRobot;
            this.shooterAngle = shooterAngle;
        }
    }

    private final Alliance alliance;
    public LookAt(Alliance alliance) {
        this.alliance = alliance;
    }

    public Result calcs(double absX, double absY, double wheelW, boolean isNear) {

        Calculations.Coords robot = new Calculations.Coords(absX, absY);

        double targetX;
        double targetY;

        if (alliance == Alliance.BLUE) {

            targetX = ConfigLookAt.blueGoalPointX;
            targetY = ConfigLookAt.blueGoalPointY;

            double rot = Calculations.pointsAngle(
                    robot,
                    new Calculations.Coords(targetX, targetY)
            );

            double shooter = Calculations.shootingTraectoryCalc(
                    robot,
                    new Calculations.Coords(targetX, targetY),
                    ConfigLookAt.goalPointHeight,
                    wheelW,
                    isNear
                            ? ConfigLookAt.nearZoneIs1stParabolaPoint
                            : ConfigLookAt.farZoneIs1stParabolaPoint
            );

            return new Result(rot, shooter);
        }

        // RED
        targetX = ConfigLookAt.redGoalPointX;
        targetY = ConfigLookAt.redGoalPointY;

        double rot = constrTo180(
                Calculations.pointsAngle(robot, new Calculations.Coords(targetX, targetY)) + 180
        );

        double shooter = Calculations.shootingTraectoryCalc(
                robot,
                new Calculations.Coords(targetX, targetY),
                ConfigLookAt.goalPointHeight,
                wheelW,
                isNear
                        ? ConfigLookAt.nearZoneIs1stParabolaPoint
                        : ConfigLookAt.farZoneIs1stParabolaPoint
        );

        return new Result(rot, shooter);
    }
}