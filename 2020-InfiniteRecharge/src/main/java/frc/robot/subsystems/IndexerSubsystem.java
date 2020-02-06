package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  static CANSparkMax[] beltMotors;
  
  // DumperCommands or BallCollectionCommands that need to move the conveyor belts
  private List<CommandBase> activeBallCommands = new ArrayList<CommandBase>();

  public IndexerSubsystem() {
    beltMotors = new CANSparkMax[Constants.IndexerConstants.kBeltIDs.length];
    for(int i=0; i<beltMotors.length; i++)
    {
      beltMotors[i] = new CANSparkMax(Constants.IndexerConstants.kBeltIDs[i], MotorType.kBrushless);
    }
  }

  public void addActiveBallCommand(CommandBase command)
  {
    activeBallCommands.add(command);
  }

  public void removeActiveBallCommand(CommandBase command)
  {
    activeBallCommands.remove(command);
  }

  @Override
  public void periodic() {
    boolean shouldMoveBelts = activeBallCommands.size() > 0;
    for(CANSparkMax motor : beltMotors)
    {
      motor.set(shouldMoveBelts ?Constants.IndexerConstants.BELT_SPEED :0);
    }
  }
}
