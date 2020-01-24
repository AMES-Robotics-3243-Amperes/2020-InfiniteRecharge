/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveTrainSubSystem {
    private CANSparkMax motor_1 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax motor_2 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax motor_3 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax motor_4 = new CANSparkMax(3, MotorType.kBrushless);

    // Left and right side drive
    private final SpeedControllerGroup m_leftmotors = new SpeedControllerGroup(motor_1, motor_3);
    private final SpeedControllerGroup m_rightmotors = new SpeedControllerGroup(motor_2, motor_4);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);

    public void tankDrive(Double[] var){
      deadZone(var);
      scaleZone(var);
      m_drive.tankDrive(var[0], var[1]);
    }
    
    public Double[] deadZone(Double[] dead){      
      
      if(-0.09 < dead[0] || -0.09 < dead[1] && 0.09 > dead[0] || 0.09 < dead[1]){
        dead[0] = 0.0;
        dead[1] = 0.0;
      }

      return dead;
    }

    public Double[] scaleZone(Double[] scale){

      scale[0] = scale[0] * Math.abs(scale[0]); //Dampens the input value by squaring it
      scale[1] = scale[1] * Math.abs(scale[1]); //Dampens the input value by squaring it

      return scale;
    }


}
