/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.commands.DriveTrainCommand;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainPIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */
public class DriveTrainSubSystem extends SubsystemBase {

  SpeedController motor1;

  Encoder encode1;

  // New Encoder Objects \\
    private static CANSparkMax motor_LT = new CANSparkMax(Constants.DriveConstants.kLTID, MotorType.kBrushless);
    private static CANSparkMax motor_LB = new CANSparkMax(Constants.DriveConstants.kLBID, MotorType.kBrushless);
    private static CANSparkMax motor_RT = new CANSparkMax(Constants.DriveConstants.kRTID, MotorType.kBrushless);
    private static CANSparkMax motor_RB = new CANSparkMax(Constants.DriveConstants.kRBID, MotorType.kBrushless);

    // Left and right side drive
    public final static SpeedControllerGroup m_leftmotors = new SpeedControllerGroup(motor_LT, motor_LB); // Classifying left side motors
    public final static SpeedControllerGroup m_rightmotors = new SpeedControllerGroup(motor_RT, motor_RB); // Classifying right side motors
    
    private static WPI_VictorSPX drive_LT = new WPI_VictorSPX(Constants.DriveConstants.kPracLTID);
    private static WPI_VictorSPX drive_LB = new WPI_VictorSPX(Constants.DriveConstants.kPracLBID);
    private static WPI_VictorSPX drive_RT = new WPI_VictorSPX(Constants.DriveConstants.kPracRTID);
    private static WPI_VictorSPX drive_RB = new WPI_VictorSPX(Constants.DriveConstants.kPracRBID);


    static boolean testBot = false;
    static double leftVector = 0.0;
    static double rightVector = 0.0;

  // m_drive is a combination of both left and right motors
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);

  // Encoder data objects
  private final static DrivetrainPIDSubsystem m_rightSide = new DrivetrainPIDSubsystem(m_rightmotors,
      motor_RT.getEncoder());
  private final static DrivetrainPIDSubsystem m_leftSide = new DrivetrainPIDSubsystem(m_leftmotors,
      motor_LT.getEncoder());

  // Command Based code requirement: enabling motors
  public DriveTrainSubSystem() {
    motor_LT.setSmartCurrentLimit(40);
    motor_LB.setSmartCurrentLimit(40);
    motor_RT.setSmartCurrentLimit(40);
    motor_RB.setSmartCurrentLimit(40);

    m_rightSide.enable();
    m_leftSide.enable();

    drive_LB.follow(drive_LT);
    drive_RB.follow(drive_RT);

    if(RobotContainer.isPractice){
      motor1 = new WPI_VictorSPX(100);
      encode1 = new Encoder(3, 4, false, EncodingType.k4X);
    } else{
      motor1 = new CANSparkMax(Constants.DriveConstants.kLBID, MotorType.kBrushless);
      //encode1 = ((CANSparkMax) motor1).getEncoder();    //I no work :(
    }
  }

  public static void tankDrive(double varLeft, double varRight) {
    leftVector = varLeft;
    rightVector = varRight;


        //m_rightSide.setSetpoint(-varRight);
        //m_leftSide.setSetpoint(varLeft);

        //drive_LT.set(ControlMode.PercentOutput, varLeft);
        //drive_RT.set(ControlMode.PercentOutput, varRight);
        
      
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Vector Left: ", leftVector);
        SmartDashboard.putNumber("Vector Right: ", rightVector);



        SmartDashboard.getNumber("VelocityMotorLT: ", motor_LT.getEncoder().getVelocity()); // Prints speed of encoder
        SmartDashboard.getNumber("VelocityMotorLB: ", motor_LB.getEncoder().getVelocity());
        SmartDashboard.getNumber("VelocityMotorRT: ", motor_RT.getEncoder().getVelocity());
        SmartDashboard.getNumber("VelocityMotorRB: ", motor_RB.getEncoder().getVelocity());
  
        SmartDashboard.getNumber("CurrentMotorLT: ", motor_LT.getOutputCurrent());  // Prints current in amps
        SmartDashboard.getNumber("CurrentMotorLB: ", motor_LB.getOutputCurrent());
        SmartDashboard.getNumber("CurrentMotorRT: ", motor_RT.getOutputCurrent());
        SmartDashboard.getNumber("CurrentMotorRB: ", motor_RB.getOutputCurrent());
  
        SmartDashboard.getNumber("CurrentMotorLT: ", motor_LT.getBusVoltage()); // Prints the voltage going into the motor controller
        SmartDashboard.getNumber("CurrentMotorLB: ", motor_LB.getBusVoltage());
        SmartDashboard.getNumber("CurrentMotorRT: ", motor_RT.getBusVoltage());
        SmartDashboard.getNumber("CurrentMotorRB: ", motor_RB.getBusVoltage());
      
      // This method will be called once per scheduler run
      

      
    }

}
