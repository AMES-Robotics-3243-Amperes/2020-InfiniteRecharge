package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmsSubsystem;
import frc.robot.subsystems.ClimbWinchSubsystem;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {

  private ClimbWinchSubsystem climbWinch;
  private ClimbArmsSubsystem climbArms;
  private final ClimbExtendArmsCommand extendArms;
  private final ClimbExtendWinchCommand extendWinch;
  private final ClimbRetractArmsCommand retractArms;
  private final ClimbRetractWinchCommand retractWinch;
  private boolean isInDeployMode = false; // Start on false, so the first activation sets it to true and schedules a ClimbExtendCommand
  private boolean areArmsScheduled = false; // makes sure arms are scheduled to extend
  public ClimbCommand(ClimbWinchSubsystem climbWinch, ClimbArmsSubsystem climbArms) {
    // This command does NOT declare requirements. extendsArms, extendsWinch, etc. declare their own requirements.
    this.climbWinch = climbWinch; // Set variable to the object
    this.climbArms = climbArms;
    this.extendArms = new ClimbExtendArmsCommand(climbArms);
    this.extendWinch = new ClimbExtendWinchCommand(climbWinch);
    this.retractArms = new ClimbRetractArmsCommand(climbArms);
    this.retractWinch = new ClimbRetractWinchCommand(climbWinch);
  }

 
  @Override
  public void initialize() {
    isInDeployMode = !isInDeployMode;
    // Decide whether to schedule extension commands or retraction commands
    if (isInDeployMode) {
      extendWinch.schedule();
      areArmsScheduled = false;  //setting arms scheduled to false after exectue runs
    } else {
      
      retractWinch.schedule();
    }
  }

  @Override
  public void execute() {
    if(  climbWinch.isWinchDeployed() && isInDeployMode ){ // checks when deploy mode is true and winch is deployed
      extendArms.schedule();
      areArmsScheduled = true;
    
    }
  }

 @Override
  public void end(boolean interrupted) {
           
}

  @Override
  public boolean isFinished() {
    return areArmsScheduled;
  }
}
