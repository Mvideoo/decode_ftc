package org.firstinspires.ftc.teamcode.v2.components.util;

import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigLookAt;
import org.firstinspires.ftc.teamcode.v2.util.dashconfigs.ConfigShooterController;

import static java.lang.Math.*;

public class Calculations {

    public static class Coords {
        public final double x;
        public final double y;

        public Coords(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class RotationCalculations {
        public final double revs;
        public final double Ro;

        public RotationCalculations(double revs, double Ro) {
            this.revs = revs;
            this.Ro = Ro;
        }
    }

    public static Coords pic2r(double alpha, double beta, double x, double y, double h, double fx, double fy, double cx, double cy) {
        double a = alpha * PI / 180;
        double b = beta * PI / 180;

        double y_s_shapochkoy = h * (1 / tan(atan((y - cy) / fy) + b));
        double x_s_shapochkoy = (y_s_shapochkoy * (x - cx)) / fx;

        double x_x0 = y_s_shapochkoy * sin(a);
        double y_x0 = y_s_shapochkoy * cos(a);

        double x_y0 = x_s_shapochkoy * cos(a);
        double y_y0 = x_s_shapochkoy * -sin(a);

        return new Coords(x_x0 + x_y0, y_x0 + y_y0);
    }

    public static Coords rotateCoordSystem(double x, double y, double alpha) {
        double a = alpha * PI / 180;

        double x_x0 = y * sin(a);
        double y_x0 = y * cos(a);

        double x_y0 = x * cos(a);
        double y_y0 = x * -sin(a);

        return new Coords(x_x0 + x_y0, y_x0 + y_y0);
    }



    public static double pointsAngle(Coords point1, Coords point2) {
        double deltX = point2.x - point1.x;
        double deltY = point2.y - point1.y;

        if (deltY == 0.0) {
            return deltX > 0 ? 90.0 : -90.0;
        }

        double angle = atan(deltX / deltY) / PI * 180;
        if (deltY < 0) {
            angle += angle < 0 ? 180 : -180;
        }

        return angle;
    }

    public static double calcParabolaX(double alpha, double V, double g, double y, boolean is1parabolaPoint) {
        double a = alpha * PI / 180;
        double s = is1parabolaPoint
                ? -sqrt(pow(sin(a) * V, 2) - 2 * g * y)
                :  sqrt(pow(sin(a) * V, 2) - 2 * g * y);

        return (cos(a) * V * (sin(a) * V + s)) / g;
    }

    public static double shootingTraectoryCalc(Coords robotPoint, Coords targetPoint, double targetPointHeight, double wheelW, boolean is1parabolaPoint) {
        double V = wheelW * ConfigLookAt.shooterWheelD * PI;
        double distToGoal = sqrt(pow(robotPoint.x - targetPoint.x, 2) + pow(robotPoint.y - targetPoint.y, 2));

        double alpha = ConfigShooterController.servoDownAngle;
        while (alpha < ConfigShooterController.servoUpAngle + 0.1) {
            if (calcParabolaX(alpha, V, 9.80665, targetPointHeight, is1parabolaPoint) > distToGoal) break;
            alpha += 0.5;
        }

        return alpha;
    }
}