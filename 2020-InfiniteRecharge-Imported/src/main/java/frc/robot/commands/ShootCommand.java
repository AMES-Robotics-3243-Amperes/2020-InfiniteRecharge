/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DumperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ShootCommand extends CommandBase {
  private DumperSubsystem shooter;
  private LimelightSubsystem limelight;

  // full motor speed (no PID) = 101 inches to frame front


  public ShootCommand(DumperSubsystem shooter, LimelightSubsystem limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if (RobotContainer.configureBallHighShoot()) {
      double inchesToHighGoal = limelight.getInchesFromHighGoal();
      double shootSpeed = 5700; // TODO: limelight finds shoot speed
      shooter.setDumpHighSpeed();
      //shooter.setShootSpeed(shootSpeed); // actual: 4300 rpm
      //shooter.setShootSpeedOnOrOff(true); // actual: 5200 rpm
    } else if (RobotContainer.configureBallLowShoot()) //goes to robotContainer to determine boolean button 8
    {
      shooter.setDumpLowSpeed(); // dumper subsystem
    } else if (RobotContainer.configureBallMedShoot()) {
      shooter.setDumpMedSpeed();
    } else {
      shooter.stopShoot();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShootSpeedOnOrOff(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
