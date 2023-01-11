package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.debug.SystemTest;
import frc.robot.utilities.swerve.SwerveModule;

public class DriveTrain extends SubsystemBase implements SystemTest{

  private static SwerveModule[] swerveModules;
  //private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;

  static {
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG);
    swerveModules[1] = new SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG);
    swerveModules[2] = new SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG);
    swerveModules[3] = new SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG);

    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);
   
    //pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions = {swerveModules[0].getPostion(), swerveModules[1].getPostion(), swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), SwervePositions);
    pose = new Pose2d();
  }

  public void init(){
   // pdh.clearStickyFaults();
    zeroHeading();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public double getRoll(){
    return gyro.getRoll(); 
  }

  public double getPitch(){
    return gyro.getPitch(); 
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModuelPostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  private SwerveModulePosition[] getModuelPostions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedforwardDrive(ChassisSpeeds speeds){
    double xSpeed = chassisSpeeds.vxMetersPerSecond + speeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond + speeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond + speeds.omegaRadiansPerSecond;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
    }
  }

  public void lockWheels(){
    double angle = Math.toRadians(getHeading()-45d);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
      swerveModules[i].setToAngle(angle);
    }
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());

    for (int i = 0; i < swerveModules.length; i++) {
       SmartDashboard.putData(swerveModules[i]);//("Swerve Module "+i, swerveModules[i].getEncoderRadians());
      // swerveModules[i].setEncoderOffset(SmartDashboard.getNumber("Swerve Offset "+i, swerveModules[i].getEncoderRadians()));
      // SmartDashboard.putNumber("Swerve Module rotation "+i, swerveModules[i].getRotationMotorPosition());

    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);


    odometry.update(getRotation2d(), getModuelPostions());
    pose = odometry.getPoseMeters();
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public boolean testFunctionality(int id, double speed) {

    if(id < 0 || id >= swerveModules.length*2) {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].stop();
      }
      return true;
    }
    
    if(id >= 4) {
      swerveModules[id-4].setRotationMotor(speed);
      //stop previous module
      swerveModules[id-5].setRotationMotor(0);
    }
    else {
      swerveModules[id].setDriveMotor(speed);
      //stop previous module
      if(id > 0) swerveModules[id-1].setDriveMotor(0);
    }
  
    return false;
  }

  @Override
  public int getNumberOfComponents() {
    return swerveModules.length*2;
  }
}


