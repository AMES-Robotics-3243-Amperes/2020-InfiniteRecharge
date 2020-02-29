package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmsSubsystem;
import frc.robot.RobotContainer;

public class ClimbExtendArmsCommand extends CommandBase {

  private final ClimbArmsSubsystem climber;

  public ClimbExtendArmsCommand(ClimbArmsSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
    climber.extendArmsForClimbing();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopArmLeft();
    climber.stopArmRight();        
  }

  @Override
  public boolean isFinished() {
    return climber.isClimberArmExtended();
  }
}
