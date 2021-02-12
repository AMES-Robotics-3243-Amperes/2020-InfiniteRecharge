/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmsSubsystem;
import frc.robot.subsystems.ClimbWinchSubsystem;
import frc.robot.util.JoystUtil;

public class ClimberJoystickCommand extends CommandBase {

  private final ClimbArmsSubsystem climbArms;
  private final ClimbWinchSubsystem climbWinch;
  private final Joystick joystick;

  /**
   * Creates a new ClimberJoystickCommand.
   */
  public ClimberJoystickCommand(ClimbArmsSubsystem climbArms, ClimbWinchSubsystem climbWinch, Joystick joyst) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbArms = climbArms;
    this.climbWinch = climbWinch;
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

    climbArms.setLeftVelocity(leftJoystick);
    climbArms.setRightVelocity(rightJoystick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
