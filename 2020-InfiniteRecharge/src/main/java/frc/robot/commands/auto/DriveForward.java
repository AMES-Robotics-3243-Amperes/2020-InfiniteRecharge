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

  public DriveForward(int rotations){/*DriveTrainSubSystem driver, CANEncoder encodeSparkLeft, CANEncoder encodeSparkRight, Encoder encodeVictorLeft, Encoder encodeVictorRight) {*/
    // Use addRequirements() here to declare subsystem dependencies.
    /*m_driver = driver;
    m_encodeSparkLeft = encodeSparkLeft;
    m_encodeVictorLeft = encodeVictorLeft;
    m_encodeSparkRight = encodeSparkRight;
    m_encodeVictorRight = encodeVictorRight; */
    this.rotations = rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrainSubSystem.resetEncode();
/*
    if(m_encodeSparkLeft != null && m_encodeSparkRight != null){
      m_encodeSparkLeft.setPosition(0);
      m_encodeSparkRight.setPosition(0);

      if(m_encodeSparkLeft.getPosition() < 35 && m_encodeSparkRight.getPosition() < 35)
        DriveTrainSubSystem.tankDrive(0.55, 0.55, false);
      else if(m_encodeSparkLeft.getPosition() > 35 && m_encodeSparkRight.getPosition() > 35)
        DriveTrainSubSystem.tankDrive(-0.55, -0.55, false);
      
    } else if(m_encodeVictorLeft != null && m_encodeVictorRight != null){
      m_encodeVictorLeft.reset();
      m_encodeVictorRight.reset();

      if(m_encodeVictorLeft.getDistance() < 35 && m_encodeVictorRight.getDistance() < 35)
        DriveTrainSubSystem.tankDrive(0.55, 0.55, false);
      else if(m_encodeVictorLeft.getDistance() > 35 && m_encodeVictorRight.getDistance() > 35)
        DriveTrainSubSystem.tankDrive(-0.55, -0.55, false);
    } else{
      DriveTrainSubSystem.tankDrive(0.0, 0.0, false);
    }
      */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Want to move 5 ft fwd, but don't know how many rotations, so used "35" as place holder
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
    return false;
  }
}
