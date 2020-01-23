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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class DriveTrainSubSystem {
    private CANSparkMax motor_1 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax motor_2 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax motor_3 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax motor_4 = new CANSparkMax(3, MotorType.kBrushless);

    public void motors(double joystickOne, double joystickTwo){
        motor_1 = set(joystickOne);
        motor_2 = set(joystickOne);
        motor_3 = set(joystickTwo);
        motor_4 = set(joystickTwo);
    }


}
