/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.JoystUtil;

public class ClimberJoystickCommand extends CommandBase {

  private final ClimbSubsystem climber;
  private final Joystick joystick;

  /**
   * Creates a new ClimberJoystickCommand.
   */
  public ClimberJoystickCommand(ClimbSubsystem climber, Joystick joyst) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.joystick = joyst;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftJoystick = joystick.getRawAxis(1); 
    double rightJoystick = joystick.getRawAxis(3);

    leftJoystick = JoystUtil.deadZone(leftJoystick);
    rightJoystick = JoystUtil.deadZone(rightJoystick);

    climber.setLeftVelocity(leftJoystick);
    climber.setRightVelocity(rightJoystick);
    System.err.println("#### Left Climb ####: " + leftJoystick);
    System.err.println("#### Right Climb ####: " + rightJoystick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
