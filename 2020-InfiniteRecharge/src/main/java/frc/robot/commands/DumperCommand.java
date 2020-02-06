/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DumperSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Spins the dumper mototr for a set amount of time
 */
public class DumperCommand extends CommandBase {
  private final DumperSubsystem dumper;
  private double startTime;
  private double runTime;

  public DumperCommand(DumperSubsystem dumper, double runTime) {  // Changed DumperSubsystem variable from "controlPanel" to "dumper"
	this.dumper = dumper;  //Sets the variable to the object
	this.runTime = runTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	startTime = Timer.getFPGATimestamp();
	dumper.setMotorSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dumper.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= startTime + runTime;
  }
}