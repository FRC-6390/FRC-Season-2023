package frc.robot.utilities.controlloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class PID implements Sendable {

    private static int instances = 0;
    private DoubleSupplier measurement, setpoint;
    private PIDConfig config;
    private double error, previousError, errorSum, previousTime;
    private boolean enabled = true;
    private double threshold = 0.0; // Threshold for acceptable error

    public PID(PIDConfig config){
        this(null, 0, config);
    }

    public PID(DoubleSupplier measurement, double setpoint, PIDConfig config){
        this(measurement, ()->setpoint, config);
    }
    
    public PID(DoubleSupplier measurement, DoubleSupplier setpoint, PIDConfig config){
        this.measurement = measurement;
        this.setpoint = setpoint;
        this.config = config;
        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);
    }

    public double calculate(double setpoint){
        setSetpoint(setpoint);
        return calculate();
    }

    public double calculate(){
        error = config.getContinuous() ? calculateContinuousError() : calculateError();
        // Check if the error is within the threshold
        if(Math.abs(error) <= threshold) {
            errorSum = 0; // Optionally reset the errorSum if you're within threshold
            return 0; // Return 0 or a nominal value indicating no further adjustment needed
        }

        double deltaTime = System.currentTimeMillis() - previousTime;
        if(Math.abs(error) < config.getILimit()) errorSum += error * deltaTime;
        double errorRate = deltaTime > 0 ? (error - previousError) / deltaTime : 0;        
        previousError = error;
        previousTime = System.currentTimeMillis();

        double p = config.getP() * error;
        double i = config.getI() * errorSum;
        double d = config.getD() * errorRate;
        double f = config.getF();
        return enabled ? p + i + d + f : 0;
    }

    private double calculateError(){
        return setpoint.getAsDouble() - measurement.getAsDouble();
    }

    private double calculateContinuousError(){
       return MathUtil.inputModulus(calculateError(), -config.getErrorBand(), config.getErrorBand());
    }

    public PID setThreshold(double threshold) {
        this.threshold = threshold;
        return this;
    }

    public boolean isDone(double threshold){
        return Math.abs(error) <= threshold;
    }

    public boolean isDone(){
        return isDone(getThreshold());
    }

    public double getThreshold() {
        return threshold;
    }

    public DoubleSupplier getSupplier(){
        return () -> calculate();
    }

   public PID setMeasurement(DoubleSupplier measurement) {
       this.measurement = measurement;
       return this;
   }

    public PID setSetpoint(double setpoint){
        this.setpoint = () -> setpoint;
        return this;
    }

    public double getSetpoint(){
        return setpoint.getAsDouble();
    }

    private boolean getEnable(){
        return enabled;
    }

    private void setEnable(boolean enabled){
        this.enabled = enabled;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //builder.setSmartDashboardType("PID");
        builder.addBooleanProperty("Enabled", this::getEnable, this::setEnable);
        builder.addDoubleProperty("P", config::getP, config::setP);
        builder.addDoubleProperty("I", config::getI, config::setI);
        builder.addDoubleProperty("D", config::getD, config::setD);
        builder.addDoubleProperty("I Limit", config::getILimit, config::setILimit);
        builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("Measurement", measurement::getAsDouble,  null);
        builder.addDoubleProperty("Calculated", () -> getSupplier().getAsDouble(), null);
    }
}
