/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;

public class DrivetrainPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new DrivetrainPID.
   */
  SpeedControllerGroup m_group;
  CANEncoder m_encoder;

  public DrivetrainPIDSubsystem(SpeedControllerGroup group, CANEncoder encoder) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.6, 9e-5, 9e-5));
        //0.8, 9e-5, 9e-4

    m_group = group;
    m_encoder = encoder;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_group.pidWrite(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getVelocity()/5676;
  }
}
