/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCollectionSubSystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.RobotContainer;

public class BallCollectionCommand extends IndexerCommand{
  /**
   * Creates a new BallCollectionCommand.
   */
  private final BallCollectionSubSystem m_bCollect;

  public BallCollectionCommand(BallCollectionSubSystem bCollect, IndexerSubsystem indexer) {
    super(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
    m_bCollect = bCollect;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_bCollect.setSpin(RobotContainer.configureballbindings());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_bCollect.setSpin(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
