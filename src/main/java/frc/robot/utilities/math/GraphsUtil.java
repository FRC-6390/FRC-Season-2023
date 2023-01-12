package frc.robot.utilities.math;

public class GraphsUtil {

    private static double isNAN(double val){
        return Double.isNaN(val) ? 0 : val;
    }

    public static double calculateLineXIntercept(double m1, double m2, double b1, double b2){
        return ((b2-b1)/(m1-m2));
    }

    public static double calculateLineYIntercept(double m1, double b1, double x){
        return m1*x+b1;
    }

    public static double calculateSlope(double x1, double x2, double y1, double y2){
        double val = (y2-y1)/(x2-x1);
        return isNAN(val);
    }

    public static double calculateSlopeYIntercept(double x, double y, double m){
        double val = y-(m*x); 
        return isNAN(val);
    }
}
