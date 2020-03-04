/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.auto.AutoMoveAndShootCommand;
import frc.robot.commands.auto.DriveForward;

import frc.robot.subsystems.DriveTrainSubSystem;
import frc.robot.subsystems.DumperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootCommandGroup extends SequentialCommandGroup {

  public ShootCommandGroup(DriveTrainSubSystem drive, LimelightSubsystem align, DumperSubsystem dump) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      // Lines up w/ limelight, then shoots @ the high goal
      new AutoMoveAndShootCommand(drive, align, dump),
      
      // Drives backwards 10 rotations to get to the trench
      new DriveForward(-10)

      // If we can, we turn around to face the trench? Not sure if want. 3/2/20
      // new DriveTurn(4)
    );
  }

}
