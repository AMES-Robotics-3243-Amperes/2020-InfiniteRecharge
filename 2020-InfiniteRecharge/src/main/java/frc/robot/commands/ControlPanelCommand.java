/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PanelColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlPanelCommand extends CommandBase {
  protected final ControlPanelSubsystem controlPanel;

  public ControlPanelCommand(ControlPanelSubsystem controlPanel) {
    this.controlPanel = controlPanel;  //Sets the variable to the object
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** @return Whether the mechanism will be lifted. If true, this command should cancel itself.
   */
  protected boolean tryLiftMechanism()
  {
    if(controlPanel.isMechanismLifted())
      return false;
    
    controlPanel.setLiftMechanism(true);
    return true;
  }

  public static class TurnNumTimes extends ControlPanelCommand
  {
    private final double rotationsMin; // If all metrics greater than this, stop.
    private final double rotationsMax; // If any metric greater than this, stop.
    private PanelColor startColor;
    private PanelColor prevColor;
    private double rotsByColor;

    public TurnNumTimes(ControlPanelSubsystem controlPanel, double rotationsMin, double rotationsMax)
    {
      super(controlPanel);
      this.rotationsMin = rotationsMin;
      this.rotationsMax = rotationsMax;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(tryLiftMechanism())
      {
        this.cancel();
        return;
      }

      System.out.println("TurnNumTimes initialize");
      startColor = PanelColor.RED; // Most reliably detected; B & Y proone to false positives, G to false negatives.
      prevColor = controlPanel.getSensorColor();
      rotsByColor = 0;
      controlPanel.resetPanelSpinnerEncoderPosition();
      // controlPanel.spinPanel(rotations); OLD
      if(Constants.TEST_VERSION)
      {
        SmartDashboard.putNumber("TurNumTimes.rotsByColor", 0);
        SmartDashboard.putNumber("TurNumTimes.Encoder Rots", 0);
      }
    }

    @Override
    public void execute()
    {
      PanelColor currentColor = controlPanel.getSensorColor();

      if(prevColor!=startColor && currentColor==startColor)
        rotsByColor += 0.5; // 2 wedges per revolution -> 1 wedge=0.5 revolutions
      controlPanel.setPanelSpinnerSpeed(this.isFinished() ?0 :1); // Rotate if we haven't rotated enough yet

      if(Constants.TEST_VERSION)
      {
        SmartDashboard.putNumber("TurNumTimes.rotsByColor", rotsByColor);
        SmartDashboard.putNumber("TurNumTimes.Encoder Rots", controlPanel.getPanelRotations());
      }

      prevColor = currentColor;
    }

    @Override
    public void end(boolean interrupted)
    {
      controlPanel.setPanelSpinnerSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      System.out.println("TurnNumTimes isFinished "+controlPanel.getPanelRotations());
      return (rotsByColor>=rotationsMin && controlPanel.getPanelRotations()>=rotationsMin) // If both metrics pass min
      || (rotsByColor>=rotationsMax || controlPanel.getPanelRotations()>=rotationsMax); // OR if either passes max
    }
  }

  public static class TurnToColor extends ControlPanelCommand
  {
    private static final double COLOR_REACHED_STABLE_TIME = 0.5;
    private final ControlPanelSubsystem.PanelColor targetColor;
    private int prevSpinDir = 0;
    private int tryIdx = 0; // How many times have we reached the target color?
    private double lastTimeReachedColor = -10000;
    private boolean isFinished = false;

    public TurnToColor(ControlPanelSubsystem controlPanel, ControlPanelSubsystem.PanelColor targetColor) {
      super(controlPanel);
      this.targetColor = targetColor;
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(tryLiftMechanism())
      {
        this.cancel();
        return;
      }
      
      prevSpinDir = 0; // reset
      tryIdx = 0;
      lastTimeReachedColor = Timer.getFPGATimestamp();
      isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      System.out.println("TurnToColor execute");

      if(isFinished)
        return;

      int spinDir = controlPanel.seekColor(targetColor, Math.pow(0.5, tryIdx));

      if(spinDir==0 && prevSpinDir!=spinDir)
        tryIdx++;
      
      if(spinDir==0) // If no spin dir, we've reached the target color
      {
        controlPanel.overrunInDir(prevSpinDir, 0);
        if(Timer.getFPGATimestamp() - lastTimeReachedColor > COLOR_REACHED_STABLE_TIME)
          isFinished = true;
      } else // We're still seeking the color
      {
        // This will start being in the past when spinDir starts being 0
        lastTimeReachedColor = Timer.getFPGATimestamp();
        //prevSpinDir = spinDir;
      }

      prevSpinDir = spinDir;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isFinished;
    }
  }

  public static class LowerMechanism extends ControlPanelCommand{
    public LowerMechanism(ControlPanelSubsystem controlPanel) {
      super(controlPanel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      controlPanel.setLiftMechanism(false);
    }

    public boolean isFinished() {
      return true; // This command only needs to run initialize()
    }
  }

  public static class Manual extends ControlPanelCommand
  {
    private final double SPEED;

    public Manual(ControlPanelSubsystem controlPanel,double speed) {
      super(controlPanel);
      addRequirements(controlPanel);
      SPEED = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(tryLiftMechanism())
      {
        this.cancel();
        return;
      }
      
      controlPanel.setPanelSpinnerSpeed(SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      controlPanel.setPanelSpinnerSpeed(0);
    }
  }
}
