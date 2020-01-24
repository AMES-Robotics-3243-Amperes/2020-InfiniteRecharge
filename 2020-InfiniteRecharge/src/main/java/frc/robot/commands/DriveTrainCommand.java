/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

/**
 * Add your docs here.
 */
public class DriveTrainCommand extends CommandBase {
    private final DriveTrainSubSystem m_drive;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    public DriveTrainCommand(DriveTrainSubSystem subsystem, DoubleSupplier x, DoubleSupplier y){
        m_drive = subsystem;
        m_x = x;
        m_y = y;
        //addRequirements(m_drive);
    }

    @Override
    public void execute(){
        m_drive.tankDrive(m_x.getAsDouble(), m_y.getAsDouble());
    }
}
