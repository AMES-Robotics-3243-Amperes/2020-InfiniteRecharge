/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.RobotContainer;

public class DriveTrainCommand extends CommandBase {
  
  private final DriveTrainSubSystem m_drive;  //This class's object of DriveTrainSubSystem
  private final LimelightSubsystem m_adjust;

  public DriveTrainCommand(DriveTrainSubSystem drive, LimelightSubsystem adjust) { //drive = actual import from DriveTrainSubSystem
    m_drive = drive;
    m_adjust = adjust;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driveLime()){
      DriveTrainSubSystem.tankDrive(
        m_adjust.setVision(RobotContainer.driveLime()),
        m_adjust.setVision(RobotContainer.driveLime())
      );
    } else if(!RobotContainer.driveLime()){
      DriveTrainSubSystem.tankDrive(
        RobotContainer.configureDriveLeft(), 
        RobotContainer.configureDriveRight()
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrainSubSystem.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
