package frc.robot;

public class EthanMath {
    public static double map(double val, double inputMin, double inputMax, double outputMin, double outputMax) {
        double x1 = inputMin;
        double x2 = inputMax;
        double y1 = outputMin;
        double y2 = outputMax;

        double m = (y2-y1)/(x2-x1);

        double b = y1 - m * x1;
        
        return m * val + b;
    }
}
