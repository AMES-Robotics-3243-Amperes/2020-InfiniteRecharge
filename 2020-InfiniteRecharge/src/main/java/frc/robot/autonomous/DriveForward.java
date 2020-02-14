/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrainSubSystem;
import frc.robot.subsystems.DrivetrainPIDSubsystem;

public class DriveForward extends CommandBase {
  /**
   * Creates a new DriveForward.
   */

   private static DriveTrainSubSystem m_driver;
   private static CANEncoder m_encodeSparkLeft;
   private static CANEncoder m_encodeSparkRight;
   private static Encoder m_encodeVictorRight;
   private static Encoder m_encodeVictorLeft; 

  public DriveForward(DriveTrainSubSystem driver, CANEncoder encodeSparkLeft, CANEncoder encodeSparkRight, Encoder encodeVictorLeft, Encoder encodeVictorRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driver = driver;
    m_encodeSparkLeft = encodeSparkLeft;
    m_encodeVictorLeft = encodeVictorLeft;
    m_encodeSparkRight = encodeSparkRight;
    m_encodeVictorRight = encodeVictorRight; 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_encodeSparkLeft != null && m_encodeSparkRight != null){
      m_encodeSparkLeft.setPosition(0);
      m_encodeSparkRight.setPosition(0);
      DriveTrainSubSystem.tankDrive(0.5, 0.5);  
    } else if(m_encodeVictorLeft != null && m_encodeVictorRight != null){
      m_encodeVictorLeft.reset();
      m_encodeVictorRight.reset();
      DriveTrainSubSystem.tankDrive(0.5, 0.5);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // We want to move 5 feet forward, so that 35 should be changed later 2/13/20
    // SparkMax Encoder code
    if(m_encodeSparkRight != null && m_encodeSparkLeft != null && m_encodeSparkRight.getPosition() <= 35 && m_encodeSparkLeft.getPosition() <= 35){
      DriveTrainSubSystem.tankDrive(0.5, 0.5);

    } else if(m_encodeSparkRight != null && m_encodeSparkLeft != null && m_encodeSparkRight.getPosition() >= 35 && m_encodeSparkLeft.getPosition() >= 35){
      DriveTrainSubSystem.tankDrive(-0.5, -0.5);

    } else if(m_encodeSparkRight != null && m_encodeSparkLeft != null && m_encodeSparkRight.getPosition() == 35 && m_encodeSparkLeft.getPosition() == 35){
      // HOW DO WE MAKE A PERFECT 90 DEGREE TURN W/0 A GYRO THEN STOP?
      DriveTrainSubSystem.tankDrive(0.25, -0.25); // Turn 90 degrees

    }

    // Victor Encoder code
    if(m_encodeVictorRight != null && m_encodeVictorLeft != null && m_encodeVictorRight.getDistance() <= 35 && m_encodeVictorLeft.getDistance() <= 35){
      DriveTrainSubSystem.tankDrive(0.5, 0.5);

    } else if(m_encodeVictorRight != null && m_encodeVictorLeft != null && m_encodeVictorRight.getDistance() >= 35 && m_encodeVictorLeft.getDistance() >= 35){
      DriveTrainSubSystem.tankDrive(-0.5, -0.5);

    } else if(m_encodeVictorRight != null && m_encodeVictorLeft != null && m_encodeVictorRight.getDistance() == 35 && m_encodeVictorLeft.getDistance() == 35){
      // SAME DEAL HERE FOR THE 90 DEGREE TURN :(((
      DriveTrainSubSystem.tankDrive(0.25, -0.25);

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
