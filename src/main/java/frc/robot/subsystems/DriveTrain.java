package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.ROBOT;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.debug.SystemTest;
import frc.robot.utilities.debug.SystemTestAction;
import frc.robot.utilities.sensors.REVBlinkin;
import frc.robot.utilities.sensors.vission.LimeLight;
import frc.robot.utilities.swerve.SwerveModule;

public class DriveTrain extends SubsystemBase implements SystemTest{

  private static SwerveModule[] swerveModules;
  private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab, autoTab;
  private static Field2d gameField;
  private static double desiredHeading;
  private static PIDConfig driftCorrectionPID = new PIDConfig(0.09, 0, 0.1).setILimit(20).setContinuous(-Math.PI, Math.PI);
  private static PID pid;
  private static LimeLight limeLight;
  private static REVBlinkin blinkin;
  private static PIDController rotationPidController = new PIDController(0.3, 0, 0);

  static {
    tab = Shuffleboard.getTab("Drive Train");
    autoTab = Shuffleboard.getTab("Auto");
    gameField = new Field2d();
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG, tab);
    swerveModules[1] = new SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);  
    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);
   
    pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions = {swerveModules[0].getPostion(), swerveModules[1].getPostion(), swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), SwervePositions);
    pose = new Pose2d();

    pid = new PID(driftCorrectionPID).setMeasurement(() -> pose.getRotation().getDegrees());
    limeLight = new LimeLight(ROBOT.LIMELIGHT_CONFIG);
    blinkin = new REVBlinkin(ROBOT.BLINKIN_PORT);

    rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void shuffleboard(){
    tab.addDouble("Front Left Encoder", () -> swerveModules[0].getAbsolutePosition());
    tab.addDouble("Front Right Encoder", () -> swerveModules[1].getAbsolutePosition());
    tab.addDouble("Back Left Encoder", () -> swerveModules[2].getAbsolutePosition());
    tab.addDouble("Back Right Encoder", () -> swerveModules[3].getAbsolutePosition());
    
    tab.addDouble("Intake Encoder", Intake::getPosition);
    tab.addBoolean("Intake Limit Switch", () -> Intake.getLimitSwitch());
    // tab.addDouble("Elevator Encoder", Elevator::getPosition);
    // tab.addBoolean("Intake Position", Intake::getIntakeBoolPosition);
    
    autoTab.addDouble("Desired Heading", () -> pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("PID Desired Heading", () -> pid.calculate(pose.getRotation().getDegrees())).withWidget(BuiltInWidgets.kTextView);

    autoTab.add(gameField);
    autoTab.addDouble("Odometry Heading", () -> pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("Odometry X", () -> pose.getX()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("Odometry Y", () -> pose.getY()).withWidget(BuiltInWidgets.kTextView);
  }

  public void init(){
    pdh.clearStickyFaults();
    zeroHeading();
    resetOdometry(pose);
  }

  public void zeroHeading(){
    gyro.setYaw(0);
    resetOdometry(pose);
  }

  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll(), 360); 
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch(),360); 
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  
  public void driftCorrection(ChassisSpeeds speeds){
    // double speed = Math.abs(getAverageSpeed()); //method removed pls refer to github if needed
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) desiredHeading = pose.getRotation().getDegrees();
    else speeds.omegaRadiansPerSecond += pid.calculate(desiredHeading);
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
    }
  }

  public void lockWheels(){
    
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
    }
    swerveModules[0].setToAngle(Math.toRadians(45));
    swerveModules[1].setToAngle(Math.toRadians(135));
    swerveModules[2].setToAngle(Math.toRadians(-45));
    swerveModules[3].setToAngle(Math.toRadians(-135));
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

  private void updateOdometry(){
    // if(limeLight.hasBotPose()){
      // if(limeLight.getPipeline() == 0){
      //   APRILTAGS tag = APRILTAGS.getByID((int)limeLight.getAprilTagID());
      //   if(!tag.equals(APRILTAGS.INVALID)){
      //     Pose2d relativePose =limeLight.getBot2DPosition();
      //     Pose2d tagPose = tag.getPose2d();
      //     pose = new Pose2d(relativePose.getX() + tagPose.getX(), relativePose.getY() + tagPose.getY(), getRotation2d());
      //   }
      // }
    // }else{
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();
    // }

    gameField.setRobotPose(pose);
  }

  public LimeLight getLimelight(){
    return limeLight;
  }

  public REVBlinkin getBlinkin(){
    return blinkin;
  }

  @Override
  public void periodic() {

    double xSpeed = chassisSpeeds.vxMetersPerSecond + feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond + feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond + feedbackSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    driftCorrection(speed);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
    
    setModuleStates(states);

    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public ArrayList<SystemTestAction> getDevices() {
    ArrayList<SystemTestAction> actions = new ArrayList<>();

    for (int i = 0; i < swerveModules.length; i++) {
      actions.add(new SystemTestAction(swerveModules[i]::setDriveMotor, 0.5));
      actions.add(new SystemTestAction(swerveModules[i]::setRotationMotor, 0.5));
    }
    return actions;
  }

}