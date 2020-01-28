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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainPIDSubsystem;


/**
 * Add your docs here.
 */
public class DriveTrainSubSystem extends SubsystemBase {
    private CANSparkMax motor_LT = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax motor_LB = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax motor_RT = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax motor_RB = new CANSparkMax(4, MotorType.kBrushless);

    // Left and right side drive
    public final SpeedControllerGroup m_leftmotors = new SpeedControllerGroup(motor_LT, motor_LB);
    public final SpeedControllerGroup m_rightmotors = new SpeedControllerGroup(motor_RT, motor_RB);
    

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);

    private final DrivetrainPIDSubsystem m_rightSide = new DrivetrainPIDSubsystem(m_rightmotors, motor_RT.getEncoder());
    private final DrivetrainPIDSubsystem m_leftSide = new DrivetrainPIDSubsystem(m_leftmotors, motor_LT.getEncoder());

    public DriveTrainSubSystem(){
      m_rightSide.enable();
      m_leftSide.enable();
    }

    public void tankDrive(Double[] var){
      //m_drive.tankDrive(var[0], var[1]);
      m_rightSide.setSetpoint(-var[1]);
      m_leftSide.setSetpoint(var[0]);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      tankDrive(
        RobotContainer.configureDriveBindings()
      );
    }

}
