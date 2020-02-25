package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {

  private final ClimbSubsystem climber;
  private final ClimbExtendCommand extend;
  private final ClimbRetractCommand retract;
  private boolean isInDeployMode = false; // Start on false, so the first activation sets it to true and schedules a ClimbExtendCommand

  public ClimbCommand(ClimbSubsystem climber, ClimbExtendCommand extend, ClimbRetractCommand retract) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
    this.extend = extend;
    this.retract = retract;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    isInDeployMode = !isInDeployMode;
    if (isInDeployMode)
      extend.schedule();
    else
      retract.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
