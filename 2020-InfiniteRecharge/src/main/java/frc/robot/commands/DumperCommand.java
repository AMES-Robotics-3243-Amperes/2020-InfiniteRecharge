/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Spins the dumper mototr for a set amount of time
 */
public class DumperCommand extends CommandBase {

  public DumperCommand() {  // Changed DumperSubsystem variable from "controlPanel" to "dumper"
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    DumperSubsystem.setDumpCollectSpeed(RobotContainer.configureBallCollect(),RobotContainer.configureBallCollectBackwards());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DumperSubsystem.setDumpCollectSpeed(false,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}