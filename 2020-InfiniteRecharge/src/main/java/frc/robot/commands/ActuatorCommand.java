// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
//Library imports
import edu.wpi.first.wpilibj2.command.CommandBase;
//class imports
import frc.robot.RobotContainer;
import frc.robot.subsystems.ActuatorSubsystem;

public class ActuatorCommand extends CommandBase {
  private ActuatorSubsystem actuation;
  
  
  /** Creates a new ActuatorCommand. */
  public ActuatorCommand(ActuatorSubsystem actuation) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.actuation = actuation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.actuatorExtend()){
      actuation.setActAngle(90);
     
    } else if (RobotContainer.actuatorRetract()){
      actuation.setActAngle(0);
      
    } else if(RobotContainer.actuatorSixty() == 90){
      actuation.setActAngle(60);
    } else {
      actuation.stopActuation(); //TODO: THIS REALLY NEEDS TO GET TESTED ASAP
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
