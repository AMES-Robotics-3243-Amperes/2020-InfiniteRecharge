package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbExtendCommand extends CommandBase {

  private final ClimbSubsystem climber;
  private boolean hasDeployed; // flag

  public ClimbExtendCommand(ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
    hasDeployed = false;
    climber.setWinchIsDeployed(true);
  }

  @Override
  public void execute() {
      if( ! hasDeployed && climber.isWinchDeployed())
      {
        climber.extendArmsForClimbing();
        hasDeployed = true;
      }
  }

  @Override
  public void end(boolean interrupted) {
    //climber.stopAllMotors();
    climber.climberWinch.stopMotor();
    climber.climberL.stopMotor();
    climber.climberR.stopMotor();
            
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchDeployed() && climber.isClimberArmExtended();
  }
}
