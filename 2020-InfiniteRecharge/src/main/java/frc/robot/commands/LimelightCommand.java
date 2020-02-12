/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubSystem;

public class LimelightCommand extends CommandBase {
  /**
   * Creates a new LimelightCommand.
   */
  private final DriveTrainSubSystem m_drive;
  private final LimelightSubsystem m_adjust;

  public LimelightCommand(DriveTrainSubSystem drive, LimelightSubsystem adjust) {
    // Use addRequirements() here to declare subsystem dependencies.
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

    double var =  0;
    double var2 = 0.0;

      //Left side

      if (LimelightSubsystem.setSteer() + LimelightSubsystem.setDist() > 1) {
        var = 1.0;
      }
      else if(LimelightSubsystem.setSteer() + LimelightSubsystem.setDist() < -1) {
        var = -1.0;
      }
      else {
        var = LimelightSubsystem.setSteer() - LimelightSubsystem.setDist();
      }
      //Right side
      if (LimelightSubsystem.setSteer() - LimelightSubsystem.setDist() > 1) {
        var2 = 1.0;
      }
      else if(LimelightSubsystem.setSteer() - LimelightSubsystem.setDist() <-1) {
        var2 = -1.0;
      }
      else {
        var2 = LimelightSubsystem.setSteer() - LimelightSubsystem.setDist();
      }

      DriveTrainSubSystem.tankDrive(var, var2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrainSubSystem.tankDrive(0,0);  //Get rid of "false" later on
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
