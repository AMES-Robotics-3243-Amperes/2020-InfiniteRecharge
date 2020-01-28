package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * A container class for SPARK MAX closed-loop PID control.
 * Use this class to control a motor in closed-loop mode.
 */
public class PIDMotor
{
	private final CANSparkMax motor;
	private final CANPIDController pidController;
	public final CANEncoder encoder;
	// 1) tune P until responds fast enough,
	// 2) tune D until stops overshooting
	// 3) increase I until "steady-state" error disappears
	private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput; // PID coefficients
	private String name;

	public PIDMotor(CANSparkMax motor)
	{
		this(motor, "M"+motor.getDeviceId());
	}

	public PIDMotor(CANSparkMax motor, String name)
	{
		this(motor, name, 0.1, 0.0001, 1, 0, 0, 1, -1);
	}

	public PIDMotor(CANSparkMax motor, double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput)
	{
		this(motor, "M"+motor.getDeviceId(), kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
	}

	public PIDMotor(CANSparkMax motor, String name, double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput)
	{
		this.motor = motor;
		this.name = name;
		this.pidController = motor.getPIDController();
		setP(kP); 
        setI(kI);
        setD(kD); 
        setIZone(kIz); 
        setFF(kFF); 
        setMaxOutput( kMaxOutput );
		setMinOutput( kMinOutput );

		encoder = motor.getEncoder();
	}

	public void setName(String name)
	{
		this.name = name;
	}

	public void setP(double p)
	{ kP = p; pidController.setP(p); }
	public double getP()
	{ return kP; }

	public void setI(double i)
	{ kI = i; pidController.setI(i); }
	public double getI()
	{ return kI; }

	public void setD(double d)
	{ kD = d; pidController.setD(d); }
	public double getD()
	{ return kD; }

	public void setIZone(double iz)
	{ kIz = iz; pidController.setIZone(iz); }
	public double getIZone()
	{ return kIz; }

	public void setFF(double ff)
	{ kFF = ff; pidController.setFF(ff); }
	public double getFF()
	{ return kFF; }

	public void setOutputRange(double min, double max)
	{ kMinOutput = min; kMaxOutput = max; pidController.setOutputRange(min, max); }

	public void setMinOutput(double min)
	{ setOutputRange(min, kMaxOutput); }
	public double getMinOutput()
	{ return kMinOutput; }

	public void setMaxOutput(double max)
	{ setOutputRange(kMinOutput, max); }
	public double getMaxOutput()
	{ return kMaxOutput; }

	/** Output PID parameters to the dashboard.*/
	public void dashboardPut()
	{
		SmartDashboard.putNumber(name+" P Gain", kP);
		SmartDashboard.putNumber(name+" I Gain", kI);
		SmartDashboard.putNumber(name+" D Gain", kD);
		SmartDashboard.putNumber(name+" I Zone", kIz);
		SmartDashboard.putNumber(name+" Feed Forward", kFF);
		SmartDashboard.putNumber(name+" Max Output", kMaxOutput);
		SmartDashboard.putNumber(name+" Min Output", kMinOutput);
	}

	/** Retrieve PID parameters from the dashboard. Call this once each periodic step. */
	public void dashboardGet()
	{
		setP(SmartDashboard.getNumber(name+" P Gain", kP));
        setI(SmartDashboard.getNumber(name+" I Gain", kI));
        setD(SmartDashboard.getNumber(name+" D Gain", kD));
        setIZone(SmartDashboard.getNumber(name+" I Zone", kIz));
        setFF(SmartDashboard.getNumber(name+" Feed Forward", kFF));
        setMaxOutput(SmartDashboard.getNumber(name+" Max Output", kMaxOutput));
        setMinOutput(SmartDashboard.getNumber(name+" Min Output", kMinOutput));
    }
    
    /**
     * Set the target motor position, in number of rotations
     */
    public void setPIDPosition(double r)
    {
        setPIDReference(r, ControlType.kPosition);
    }

    /**
     * Set the target motor speed. TODO: in what units?
     */
    public void setPIDVelocity(double v)
    {
        setPIDReference(v, ControlType.kVelocity);
    }

    /**
     * A wrapper for PIDMotor.pidController.setReference. Used to set target values for closed-loop control.
     */
    public void setPIDReference(double value, ControlType ctrlType)
    {
        pidController.setReference(value, ctrlType);
    }
}