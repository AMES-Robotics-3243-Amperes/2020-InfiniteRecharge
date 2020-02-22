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
  /**
   * Creates a new AssimilatorCommand.
   */
  private boolean retract = false;
   
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(IntakeSubsystem.currentExtended){
      retract = true;
    } else if(IntakeSubsystem.currentRetracted){
      retract = false;
    } else {
      retract = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(retract){
      IntakeSubsystem.setRetract();
    } else{
      IntakeSubsystem.setExtend();
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
