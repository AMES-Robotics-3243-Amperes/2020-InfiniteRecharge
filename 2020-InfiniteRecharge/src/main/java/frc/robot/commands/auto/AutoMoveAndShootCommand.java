/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveTrainSubSystem;
import frc.robot.subsystems.DumperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AutoMoveAndShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubSystem driveTrain;
  private final LimelightSubsystem limeAlign;
  private final DumperSubsystem shooter;
  private boolean hasAligned = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoMoveAndShootCommand(DriveTrainSubSystem driveTrain, LimelightSubsystem limeAlign, DumperSubsystem shooter) {
    this.driveTrain = driveTrain;
    this.limeAlign = limeAlign;
    this.shooter = shooter;
    addRequirements(driveTrain);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasAligned = false;

    limeAlign.setPIDSteer();
    limeAlign.setPIDDist();

    if(limeAlign.setPIDSteer() == 0.0 && limeAlign.setPIDDist() == 0.0)
      hasAligned = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: steer with limelight, set isAligned to whether or not we're good to shoot

    if(hasAligned)
    {
      shooter.shootBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Uncomment later 2/29/20
    //driveTrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
