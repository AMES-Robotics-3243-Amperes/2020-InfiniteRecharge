package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.JoystUtil;
import frc.robot.RobotContainer;

public class ClimbManualCommand extends CommandBase {
  
  private final ClimbSubsystem climber;
  private final Joystick joyst;
  private static final double INTERRUPT_OFFSET_MULTI = 5;


  public ClimbManualCommand(ClimbSubsystem climber, Joystick controllerJoyst) {
      // Do NOT require the climber subsystem; this command should always be running, and shouldn't be interrupted by other climber commands.
    this.climber = climber;  // Set variable to the object
    this.joyst = controllerJoyst;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      double leftDead = JoystUtil.deadZone(joyst.getRawAxis(1));
      double rightDead = JoystUtil.deadZone(joyst.getRawAxis(3));
      double left = JoystUtil.matchZone(leftDead, rightDead);
      double right = JoystUtil.matchZone(rightDead, leftDead);
      left = JoystUtil.scaleZone(left);
      right = JoystUtil.scaleZone(right);

      if(left!=0 || right!=0)
      {
          // If any other command is using the climber, cancel it.
        Command cmdToInterrput = climber.getCurrentCommand();
        if(cmdToInterrput != null && cmdToInterrput != this)
            cmdToInterrput.cancel();

        double leftOffset = left * INTERRUPT_OFFSET_MULTI;
        double rightOffset = right * INTERRUPT_OFFSET_MULTI;
        double avgOffset = (right+left)/2.0 * INTERRUPT_OFFSET_MULTI;
        if((left<0 || right<0) && ! (left>0 || right>0) // If trying to retract
                && climber.isClimberArmRetracted() // AND both arms are retracted
                && ! climber.isWinchRetracted()) // AND the winch is NOT down
            climber.setWinchTarget(climber.getWinchPosition() + avgOffset);
        else if((left<0 || right<0) && ! (left>0 || right>0) // If trying to extend
                && ! climber.isWinchDeployed()) // AND the winch is NOT up
            climber.setWinchTarget(climber.getWinchPosition() + avgOffset);
        else if(climber.isWinchDeployed())
        {
            climber.setLeftExtendTarget(climber.getLeftArmPosition() + leftOffset);
            climber.setRightExtendTarget(climber.getRightArmPosition() + rightOffset);
        }
      }
  }

  @Override
  public void end(boolean interrupted) {
      System.err.println(" *** ClimbManualCommand STOPPED ***");
    climber.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
