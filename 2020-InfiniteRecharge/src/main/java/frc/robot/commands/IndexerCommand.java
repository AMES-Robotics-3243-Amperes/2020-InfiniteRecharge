/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.RobotContainer;

public abstract class IndexerCommand extends CommandBase {
  
  private final IndexerSubsystem indexer;

  public IndexerCommand(IndexerSubsystem indexer) {
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.addActiveBallCommand(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.removeActiveBallCommand(this);
  }
}
