/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private boolean retract = false;
   
  public IntakeCommand(IntakeSubsystem intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    retract = !retract;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(retract){
      intake.setRetract();
    } else{
      intake.setExtend();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(IntakeSubsystem.currentExtended && !retract){
      return true;
    } else if(IntakeSubsystem.currentRetracted && retract){
      return true;
    } else {
      return false;
    }

  }
}
