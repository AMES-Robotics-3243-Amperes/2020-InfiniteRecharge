package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmsSubsystem;
import frc.robot.subsystems.ClimbWinchSubsystem;

/** Run at robot startup to move arms & winch to their starting positions, based on limit switch input.
 * Resets encoder positions once starting positions are reached.
 */
public class ClimbResetCommand extends CommandBase {
  
  private final ClimbArmsSubsystem climbArms;
  private final ClimbWinchSubsystem climbWinch;
  private boolean isDone = false;
  private boolean  isDoneL = false;
  private boolean isDoneR = false;
  private boolean isDoneW = false; 


  public ClimbResetCommand(ClimbArmsSubsystem climbArms, ClimbWinchSubsystem climbWinch) {
    addRequirements(climbArms);
    addRequirements(climbWinch);
    this.climbArms = climbArms;
    this.climbWinch = climbWinch;
  }

  @Override
  public void initialize() {
      if( ! isDone)
      {
        climbArms.areEncodersReset = false;
        climbWinch.areEncodersReset = false;
      }
  }

  @Override
  public void execute() {
      if( ! isDone)
      {
        isDoneL = climbArms.resetRetractLeft();
        isDoneR = climbArms.resetRetractRight();
        isDoneW = climbWinch.resetRetractWinch();

        isDone = isDoneL && isDoneR && isDoneW;

        if(isDone)
        {
            climbArms.stopAllMotors();
            climbWinch.stopAllMotors();
            climbArms.areEncodersReset = true;
            climbWinch.areEncodersReset = true;
        }
      }
      SmartDashboard.putBoolean("ClimbReset isDone", isDone);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
