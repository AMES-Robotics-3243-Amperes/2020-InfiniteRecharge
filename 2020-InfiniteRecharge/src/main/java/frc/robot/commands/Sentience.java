/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// This is the autonomous class
/*
What is needed:
Drivetrain subsystem class: robot.DriveTrainSubSystem
  left motors: 


*/
package frc.robot.commands;
// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;
import frc.robot.subsystems.SentientSubsystem;
import frc.robot.commands.DriveTrainCommand; 

public class Sentience extends CommandBase {
  private final SentientSubsystem sentientSubsystem;
  double m_time;
  public Sentience(SentientSubsystem sentientSubsystem, double time) { // Scope has the amount of time we want it to run
    this.sentientSubsystem = sentientSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    m_time = time; 
    //requires(robot.DriveTrainSubSystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robot.DriveTrainSubSystem.m_rightmotors(1.0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
