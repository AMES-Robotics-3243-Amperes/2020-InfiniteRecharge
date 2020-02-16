/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AssimilatorSubsystem;
public class AssimilatorCommand extends CommandBase {
  /**
   * Creates a new AssimilatorCommand.
   */
  AssimilatorSubsystem index = new AssimilatorSubsystem();

  public AssimilatorCommand(AssimilatorSubsystem index) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AssimilatorSubsystem.setIndexCollectSpeed(RobotContainer.configureballbindings());
    AssimilatorSubsystem.ballIndex(RobotContainer.configureIndexShaft());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AssimilatorSubsystem.setIndexCollectSpeed(false);
    AssimilatorSubsystem.ballIndex(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
