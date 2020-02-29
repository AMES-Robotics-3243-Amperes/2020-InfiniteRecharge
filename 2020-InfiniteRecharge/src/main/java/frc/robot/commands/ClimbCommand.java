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

  public ClimbCommand(ClimbWinchSubsystem climbWicnch, ClimbArmsSubsystem climbArms) {
    // This command does NOT declare requirements. extendsArms, extendsWinch, etc. declare their own requirements.
    this.climbWinch = climbWicnch; // Set variable to the object
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
      extendArms.schedule();
      extendWinch.schedule();
    } else {
      retractArms.schedule();
      retractWinch.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
