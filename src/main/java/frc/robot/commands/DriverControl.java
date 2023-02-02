package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;

public class DriverControl extends CommandBase {

  private DriveTrain driveTrain;
  private DoubleSupplier xInput, yInput, thetaInput;
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  public DriverControl(DriveTrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.lockWheels();
  }

  @Override
  public void execute() {
  
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

    // xSpeed = 0.5;
    // ySpeed = 0;
    // thetaSpeed = 0;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, -thetaSpeed, driveTrain.getRotation2d());

    driveTrain.drive(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}



// package frc.robot.commands;

// import java.sql.Driver;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Constants.DRIVETRAIN;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.utilities.controller.DebouncedButton;
// import frc.robot.utilities.controller.ModifiedAxis;
// import frc.robot.utilities.swerve.SwerveModule;

// public class DriverControl extends CommandBase {

//     private final DriveTrain swerveSubsystem;
//     private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
//     private final BooleanSupplier fieldOrientedFunction;
//     private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
//     public static SwerveDriveKinematics kinematics;

//     public DriverControl(DriveTrain swerveSubsystem,
//     DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
//             Supplier<Boolean> fieldOrientedFunction) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.xSpdFunction = xSpdFunction;
//         this.ySpdFunction = ySpdFunction;
//         this.turningSpdFunction = turningSpdFunction;
//         this.fieldOrientedFunction = fieldOrientedFunction;
//         this.xLimiter = new SlewRateLimiter(Constants.SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
//         this.yLimiter = new SlewRateLimiter(Constants.SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
//         this.turningLimiter = new SlewRateLimiter(Constants.SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);

//         kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         // 1. Get real-time joystick inputs
//         double xSpeed = xSpdFunction;
//         double ySpeed = ySpdFunction.get();
//         double turningSpeed = turningSpdFunction.get();

//         // 3. Make the driving smoother
//         xSpeed = xLimiter.calculate(xSpeed) * Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
//         ySpeed = yLimiter.calculate(ySpeed) * Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
//         turningSpeed = turningLimiter.calculate(turningSpeed)
//                 * Constants.SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

//         // 4. Construct desired chassis speeds
//         ChassisSpeeds chassisSpeeds;
//         if (fieldOrientedFunction.get()) {
//             // Relative to field
//             chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
//         } else {
//             // Relative to robot
//             chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
//         }

//         // 5. Convert chassis speeds to individual module states
//         SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

//         // 6. Output each module states to wheels
//         swerveSubsystem.setModuleStates(states);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopWheels();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
