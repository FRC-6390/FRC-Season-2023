// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Intake;

// public class IntakeControl extends CommandBase {

//   private Intake intake;
//   private double speed;
  
//   public IntakeControl(Intake intake, double speed) {
//     this.intake = intake;
//     this.speed = speed;
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     intake.setIntake(speed);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     intake.setIntake(0);
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
