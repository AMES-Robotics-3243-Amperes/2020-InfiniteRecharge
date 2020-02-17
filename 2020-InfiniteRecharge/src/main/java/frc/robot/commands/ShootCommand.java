/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DumperSubsystem;

public class ShootCommand extends CommandBase {
  /**
   * Creates a new ShootCommand.
   */

  DumperSubsystem m_dump = new DumperSubsystem();

  public ShootCommand(DumperSubsystem dump) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dump = dump;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DumperSubsystem.setDumpShootSpeed(RobotContainer.configureshootbindings());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DumperSubsystem.setDumpShootSpeed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
