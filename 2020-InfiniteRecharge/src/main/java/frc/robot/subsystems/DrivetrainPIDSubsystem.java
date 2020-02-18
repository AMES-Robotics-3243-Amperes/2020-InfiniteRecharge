/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;

public class DrivetrainPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new DrivetrainPID.
   */
  SpeedControllerGroup m_group;
  CANEncoder m_encoderSpark;
  Encoder m_encoderVictor;

  public DrivetrainPIDSubsystem(SpeedControllerGroup group, CANEncoder encoderSpark, Encoder encoderVictor) {
    // Commented out temporarily because of compile errors - Silas 2020 Jan 31
    super(
        // The PIDController used by the subsystem
        new PIDController(0.75, 1e-6, 1e-4)); // P I D // P In PID being used for correcting oscillation
        //0.8, 9e-5, 9e-4
        //0.6, 9e-10, 9e-5  What we're using right now. A tad slow
        // Latest: 0.75, 1e-6, 1e-4
      
    m_group = group;
    m_encoderSpark = encoderSpark;
    m_encoderVictor = encoderVictor;

    if(m_encoderVictor != null){
      m_encoderVictor.setDistancePerPulse(1.0 / 76.25);
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_group.pidWrite(output); 
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    if(m_encoderSpark != null){
      return m_encoderSpark.getVelocity()/5676;
    }else if(m_encoderVictor != null){
      return m_encoderVictor.getRate()/5676;
    } else{
      return 0;
    }
  }
}

