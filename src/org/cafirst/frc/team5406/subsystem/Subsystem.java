package org.cafirst.frc.team5406.subsystem;

import org.cafirst.frc.team5406.util.Gearbox;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class Subsystem {
	
	protected Gearbox gearbox;
	
	/**
	 * Constructor for a subsystem
	 * @param gearbox The primary gearbox where all the values exist.
	 */
	public Subsystem(Gearbox gearbox)
	{
		this.gearbox = gearbox;
	}
	
	/**
	 * Initializes CANTalon motor and sets all the values for the motor.
	 * @param id The ID of the Talon
	 * @param isMaster True if Talon is the master
	 * @param follow The id of the Master Talon if the Talon is a follower.
	 * @param subsystem The gearbox that cantains the desired values
	 * @return The initialized motor.
	 */
	protected CANTalon initializeMotors(int id, boolean isMaster, int follow, Gearbox subsystem)
	{
		CANTalon motor = new CANTalon(id);
		
		motor.configNominalOutputVoltage(0,  0);
		motor.configPeakOutputVoltage(12, -12);
		
		if(isMaster)
		{
			motor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			motor.reverseSensor(subsystem.isReverseEncoder());
			motor.setProfile(0);
			motor.setF(subsystem.getPID_F());
			motor.setP(subsystem.getPID_P());
			motor.setI(subsystem.getPID_I());
			motor.setD(subsystem.getPID_D());
			motor.changeControlMode(subsystem.getMode());
			motor.setInverted(subsystem.isInvert());
		}else{
			motor.changeControlMode(TalonControlMode.Follower);
			motor.set(follow);
		}
		return motor;
	}
	
	/**
	 * Initializes CANTalon motor and sets all the values for the motor using the primary gearbox.
	 * @param id The ID of the Talon
	 * @param isMaster True if Talon is the master
	 * @param follow The id of the Master Talon if the Talon is a follower.
	 * @return The initialized motor.
	 */
	protected CANTalon initializeMotors(int id, boolean isMaster, int follow)
	{
		return initializeMotors(id, isMaster, follow, gearbox);
	}

}
