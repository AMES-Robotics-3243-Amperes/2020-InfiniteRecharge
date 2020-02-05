/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  
  private final ClimbSubsystem m_elevator;

  public ClimbCommand(ClimbSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimbSubsystem.climbSensor.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimbSubsystem.setClimb(
      RobotContainer.configuregyrobindings()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbSubsystem.setClimb(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
