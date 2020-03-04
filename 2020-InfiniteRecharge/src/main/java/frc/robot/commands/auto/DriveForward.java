/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrainSubSystem;

public class DriveForward extends CommandBase {
  /**
   * Creates a new DriveForward.
   */

   /*private static DriveTrainSubSystem m_driver;
   private static CANEncoder m_encodeSparkLeft;
   private static CANEncoder m_encodeSparkRight;
   private static Encoder m_encodeVictorRight;
   private static Encoder m_encodeVictorLeft; */

   int rotations = 0;

  public DriveForward(int rotations){
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotations = rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrainSubSystem.resetEncode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrainSubSystem.setPosition(rotations, rotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrainSubSystem.tankDrive(0.0, 0.0, false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return Math.abs(DriveTrainSubSystem.readLeftEncode()) >= Math.abs(rotations) - 2 
      && - Math.abs(DriveTrainSubSystem.readRightEncode()) <= - Math.abs(rotations) + 2;
  }
}
