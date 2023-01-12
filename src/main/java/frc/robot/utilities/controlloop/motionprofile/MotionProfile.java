package frc.robot.utilities.controlloop.motionprofile;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.controlloop.PID;

public class MotionProfile {

    private MotionProfileConfig config;
    private ArrayList<MotionProfileState> states;
    private PID pid;

    public MotionProfile(MotionProfileConfig config){
        this.config = config;
    }

    public void calculate(MotionProfileState currentState, double setpoint){
        states = calculateOptimalSpeedsAndTime(currentState, setpoint, config);

        states = calculateAcclerationRamps(states);

        Collections.reverse(states);
    }

    public void init(DoubleSupplier currentPose){
        pid = new PID(currentPose, null, config.PIDConfig());
    }

    public ArrayList<MotionProfileState> calculateOptimalSpeedsAndTime(MotionProfileState currentState,  double setpoint, MotionProfileConfig config){
        ArrayList<MotionProfileState> states = new ArrayList<>();

        double d = setpoint - currentState.startingPose();
        double a = config.maxAccelerationMeters();

        double vf = Math.pow(2*a*d, 0.5);
        MotionProfileComponent comp = new MotionProfileComponent(0, 0, d, a, 0, 0, vf);
    

        states.add(currentState);
        states.add(new MotionProfileState(comp));
       
        states.add(MotionProfileState.empty());
        return states;
    }

    public MotionProfileState recalculateState(MotionProfileComponent comp, double vi){
        comp.vi = vi;
        double vf = Math.pow(Math.pow(vi,2) + 2*comp.a*comp.d,0.5);
        comp.vf = vf;

        return new MotionProfileState(comp);
    }

    public MotionProfileComponent timeStretch(MotionProfileComponent comp, double time){
        double t = time - comp.timestamp;
        // double vf = ((2*comp.d)/t ) - comp.vi;
       // comp.vf = vf;
        comp.t = t;
        return comp;
    }

    public ArrayList<MotionProfileState> calculateAcclerationRamps(ArrayList<MotionProfileState> states){
        ArrayList<MotionProfileState> modifiedStates = new ArrayList<>();
        double timestamp = 0;

        double pose = 0;
        for (int i = 0; i < states.size(); i++) {
            MotionProfileState start = states.get(i);
            MotionProfileState end = states.size() <= i+1 ? MotionProfileState.empty() : states.get(i+1);
            ArrayList<MotionProfileComponent> comps = getPathComponentValues(start.comps().get(0), end.comps().get(0));

            double t = 0;
            for (int j = 0; j < comps.size(); j++) {
                MotionProfileComponent state = comps.get(j);
                state.timestamp = t;
                t += state.t;
            }

            double d = 0;
         
            for (MotionProfileComponent state : comps) {
                state.a = solveAcceleration(state.d, state.vi, state.t);
                state.startingPose = d;
                d += state.d;
            }

            int index = states.indexOf(end);
            if(index >= 0){
                MotionProfileState state = recalculateState(end.comps().get(0), comps.get(comps.size()-1).vf);
                states.set(index, state);
            }
            
            comps.add(MotionProfileComponent.empty(d, t));
            
            Collections.reverse(comps);

            modifiedStates.add(new MotionProfileState(pose, timestamp, comps));
            timestamp += t;
            pose += d;
        }
        return modifiedStates;
    }

    public ArrayList<MotionProfileComponent> getPathComponentValues(MotionProfileComponent start, MotionProfileComponent end){
        ArrayList<MotionProfileComponent> states = new ArrayList<>();
        double m1, m2;
        double b1, b2;
        System.out.println("a:"+start.a);
        System.out.println("vf:"+start.vf);
        System.out.println("vi:"+start.vi);
        double t1 = Math.abs((start.vf - start.vi) / start.a);

        m1 = calculateSlope(0.0, t1, start.vi, start.vf);
        b1 = calculateSlopeYIntercept(0.0, start.vi, m1);

        double endvf = (start.vf * end.vf <= 0) ? 0 : end.vf;
        double t2 = Math.abs((endvf - start.vf) / start.a);
        double t3 = calculateTime(start.d, start.a, start.vi);
        System.out.println("endvf:"+endvf);
        System.out.println("t1:"+t1);
        System.out.println("t2:"+t2);
        System.out.println("t3:"+t3);

        m2 = calculateSlope(t3-t2, t3, start.vf, endvf);
        b2 = calculateSlopeYIntercept(t3-t2,start.vf, m2);

        //System.out.println(t2);

        double xInt = calculateLineXIntercept(m1, m2, b1, b2);
        //xInt += start.d;
        double yInt = calculateLineYIntercept(m1, b1, xInt);
        yInt = Double.isNaN(yInt) ? 0 : yInt;

        System.out.println( "y="+m1 + "(x)+"+ b1 + "|y="+m2 + "(x)+"+ b2);

        System.out.println("yint:"+yInt);

        if(distanceAtMaxSpeed(start.vi, yInt, xInt)  > Math.abs(start.d)){
            double vf2 = Math.pow(start.vi, 2) + 2*m1*start.d;
            yInt = Math.pow(vf2, 0.5);
           
        }
        System.out.println("yint:"+yInt);
        if(Math.abs(yInt) >= Math.abs(start.vf)){
            yInt = start.vf;
        }

        if(Math.abs(endvf) >= Math.abs(yInt)){
            endvf = yInt;
        }
        System.out.println("endvf:"+endvf);
        System.out.println("yint:"+yInt);

        MotionProfileComponent forward;
        MotionProfileComponent sustained;
        MotionProfileComponent reverse;

        double tf = Math.abs(timeAtMaxSpeed(yInt, start.a, start.vi));
        System.out.println("tf:"+tf);
        System.out.println("a:"+start.a);
        System.out.println("vi:"+start.vi);
        System.out.println("d:"+start.d);
        double df = distanceAtMaxSpeed(start.vi, yInt, tf);
        if(df != 0){
            yInt = calculateFinalVelocity(start.vi, df, tf);
        }
        forward = new MotionProfileComponent(0, 0, df, 0, tf, start.vi, yInt);
        states.add(forward);
        System.out.println("yint:" +yInt);
        double ds = start.d - df;
        System.out.println("ds:" +ds);
        
        if(Math.abs(ds) > 0.001){
            double tr = Math.abs(timeAtMaxSpeed(endvf, start.a, yInt));
            double dr = distanceAtMaxSpeed(yInt, endvf, tr);
            reverse = new MotionProfileComponent(0, 0, dr, 0, tr, yInt, endvf);
            
            ds -= dr;

            if(Math.abs(ds) > 0.001){
                double ts = Math.abs(solveTime(ds, yInt, yInt));
                sustained = new MotionProfileComponent(0, 0, ds, Double.NaN, ts, yInt, yInt);
                states.add(sustained);
            }
            
            states.add(reverse);
        }        
        
        return states;
    }

    private double calculateLineXIntercept(double m1, double m2, double b1, double b2){
        return ((b2-b1)/(m1-m2));
    }

    private double calculateLineYIntercept(double m1, double b1, double x){
        return m1*x+b1;
    }

    private double calculateSlope(double x1, double x2, double y1, double y2){
        double value = (y2-y1)/(x2-x1);
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateSlopeYIntercept(double x, double y, double m){
        double value = y-(m*x); 
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateFinalVelocity(double vi, double d, double t){
        double value = (2*d)/t - vi;
        return Double.isNaN(value) ? 0 : value;
    }

    public double solveAcceleration(double d, double vi, double t){
        double value = (2*(d-vi*t))/(t*t);
        return Double.isNaN(value) ? 0 : value;

    }

    private double timeAtMaxSpeed(double vf, double a, double vi){
        double val = (vf - vi)/a;
        return Double.isNaN(val) ? 0 : val;
    }

    private double distanceAtMaxSpeed(double vi, double vf, double t){
        double val = (vi + vf)/2 * t;
        return Double.isNaN(val) ? 0 : val;
    }

    public boolean endOfPath(Pose2d pose){
        return false;
    }

    private MotionProfileState findRelaventState(double time){
        for (int i = 0; i < states.size(); i++) {
            MotionProfileState state = states.get(i);
            if(state.timestamp() > time) continue;
            return state;
        }
        return states.get(states.size()-1);
    }

    public double getSpeedsAtTime(double time){
        double speed = findRelaventState(time).getSpeed(time);
        speed += pid.calculate(getPoseAtTime(time));
        return speed;
    }

    public double getPoseAtTime(double time){
        return findRelaventState(time).getPoseAtTime(time);
    }

    private double solveTime(double d, double vi, double vf){
        double value =  d /((vi+vf)/2);
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateTime(double d, double a, double vi){
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
        return Double.isNaN(root) ? 0 : root; 
    }
}



