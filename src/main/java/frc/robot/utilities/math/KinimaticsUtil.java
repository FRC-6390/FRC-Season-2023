package frc.robot.utilities.math;

public class KinimaticsUtil {

    private static double isNAN(double val){
        return Double.isNaN(val) ? 0 : val;
    }

    public static double calculateFinalVelocity(double vi, double d, double t){
        double val = (2*d)/t - vi;
        return isNAN(val);
    }

    public static double solveAcceleration(double d, double vi, double t){
        double val = (2*(d-vi*t))/(t*t);
        return isNAN(val);
    }

    public static double timeAtMaxSpeed(double vf, double a, double vi){
        double val = (vf - vi)/a;
        return isNAN(val);
    }

    public static double distanceAtMaxSpeed(double vi, double vf, double t){
        double val = (vi + vf)/2 * t;
        return isNAN(val);
    }

    public static double solveTime(double d, double vi, double vf){
        double val =  d /((vi+vf)/2);
        return isNAN(val);
    }

    public static double calculateTime(double d, double a, double vi){
        double root1, root2, root;
        double determinant = vi * vi - 4 * a * -d;
        if (determinant == 0.0){
            root = -vi/(2.0*a);
           
        }else if(determinant > 0){
            root1 = (-vi + Math.sqrt(determinant)) / (2.0 * a);
            root2 = (-vi - Math.sqrt(determinant)) / (2.0 * a);
            root = root1 > root2 ? root1 : root2;
        }else{
            root = 0;
        }
        return isNAN(root);
    }
}
