package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.auto.JanusRoute;

public class JanusAuto extends CommandBase {

    private DriveTrain driveTrain;
    private JanusRoute route;

    public JanusAuto(DriveTrain driveTrain, JanusRoute route) {
        this.driveTrain = driveTrain;
        this.route = route;
        addRequirements(driveTrain);
    }
  
    @Override
    public void initialize() {
        route.init(driveTrain::getPose);
    }
  
    @Override
    public void execute() {

        if(route.isCommand()){
            route.runCommand();
        }else{
            
        }
        driveTrain.drive(route.calculate());
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }