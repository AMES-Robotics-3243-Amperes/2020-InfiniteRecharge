/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public void tankDrive(double Left, double Right){
      m_drive.tankDrive(Left, Right);
    }
    


}
