package frc.robot.utilities.controlloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

public class PID implements Sendable {

    private static int instances = 0;
    private DoubleSupplier measuremnt, setpoint;
    private PIDConfig config;
    private double error, previousError, errorSum, previousTime;

    public PID(DoubleSupplier measuremnt, double setpoint, PIDConfig config){
        this(measuremnt, ()->setpoint, config);
    }
    
    public PID(DoubleSupplier measuremnt, DoubleSupplier setpoint, PIDConfig config){
        this.measuremnt = measuremnt;
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
        double deltaTime = Timer.getFPGATimestamp() - previousTime;
        if(Math.abs(error) < config.getiLimit()) errorSum += error *config.getI();
        double errorRate = (error - previousError) / deltaTime;        
        previousError = error;
        previousTime = Timer.getFPGATimestamp();

        return config.getP()*error + config.getI()*errorSum + config.getD()*errorRate;
    }

    private double calculateError(){
        return setpoint.getAsDouble() - measuremnt.getAsDouble();
    }

    private double calculateContinuousError(){
       return MathUtil.inputModulus(calculateError(), -config.getErrorBand(), config.getErrorBand());
    }

    public DoubleSupplier getSupplier(){
        return () -> calculate();
    }

    public PID setSetpoint(double setpoint){
        this.setpoint = () -> setpoint;
        return this;
    }

    public double getSetpoint(){
        return setpoint.getAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PID");
        builder.addDoubleProperty("measurement", measuremnt::getAsDouble,  null);
        builder.addDoubleProperty("value", () -> getSupplier().getAsDouble(), null);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
