/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;

public class DrivetrainPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new DrivetrainPID.
   */
  public DrivetrainPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.6255, 5.225e-34, 7.385e-153));
        //0.7, 1e-20, 1e-300
        //0.36, 1.55e-35, 9.35e-152
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    RobotContainer.m_robotDriveSubsystem.m_rightmotors.pidWrite(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return RobotContainer.m_robotDriveSubsystem.getRightSpeed();
  }
}
