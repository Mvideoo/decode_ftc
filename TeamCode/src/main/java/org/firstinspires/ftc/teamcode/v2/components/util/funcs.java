package org.firstinspires.ftc.teamcode.v2.components.util;

import java.util.ArrayList;

public class funcs {



    public static ArrayList<Double> normToMax(ArrayList<Double> values, double maxVal) {
        ArrayList<Double> result = new ArrayList<>();

        double sum = 0.0;
        for (double v : values) {
            sum += Math.abs(v);
        }

        if (sum <= maxVal) {
            return values;
        }

        double scale = maxVal / sum;

        for (double v : values) {
            result.add(Math.abs(v) * scale * Math.signum(v));
        }

        return result;
    }

    public static double constrTo180(double value) {
        if (value > 180) return value - 360;
        if (value < -180) return value + 360;
        return value;
    }

    public static double map(
            double value,
            double min1, double max1,
            double min2, double max2
    ) {
        double zeroToOne = (value - min1) / (max1 - min1);
        return (max2 - min2) * zeroToOne + min2;
    }
}