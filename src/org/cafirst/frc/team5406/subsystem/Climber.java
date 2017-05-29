package org.cafirst.frc.team5406.subsystem;

import org.cafirst.frc.team5406.util.Constants;

import com.ctre.CANTalon;

public class Climber extends Subsystem {

	private CANTalon climberMasterMotor;
	private CANTalon climberSlaveMotor;
	
	private int direction = 1;
	private boolean isDirectionSwitched = false;
	
	private int climberCounter = 0;
	
	public Climber() 
	{
		super(Constants.climber);
		
		climberMasterMotor = initializeMotors(Constants.CLIMBER_MASTER_ID, true, 0);
		climberSlaveMotor = initializeMotors(Constants.CLIMBER_SLAVE_ID, false, Constants.CLIMBER_MASTER_ID);
		
		climberMasterMotor.setCurrentLimit(30);
		climberSlaveMotor.setCurrentLimit(30);
		climberMasterMotor.EnableCurrentLimit(true);
		climberSlaveMotor.EnableCurrentLimit(true);
	}
	
	/**
	 * Spins the climber at the desired rpm. Switches directions if current output is too great
	 * @param rpm The desired rpm
	 */
	public void climb(double rpm)
	{
		if (climberMasterMotor.getOutputCurrent() > 30 && !isDirectionSwitched)
		{
			direction *= -1;
			isDirectionSwitched = true;
		}
		
		if(climberMasterMotor.getOutputCurrent() < 20)
			climberCounter++;
		
		if(climberCounter > 20)
		{
			climberMasterMotor.EnableCurrentLimit(false);
			climberSlaveMotor.EnableCurrentLimit(false);
		}
		
		climberMasterMotor.set(direction * rpm);
	}
	
	/**Spins the climber at the target rpm. Switches directions if current output is too great*/
	public void climb()
	{
		climberMasterMotor.enable();
		climb(Constants.climber.getTarget());
	}

	/**
	 * Spins the climber in reverse at the desired rpm
	 * @param rpm The desired rpm
	 */
	public void climbReverse(double rpm)
	{
		climberMasterMotor.set(-1 * direction * rpm);
	}
	
	/**Spins the climber in reverse at the target rpm*/
	public void climbReverse()
	{
		climbReverse(Constants.climber.getTarget());
	}
	
	/**Stops the climber from spinning*/
	public void climbStop(){
		climberMasterMotor.set(0);
	}
	
	public void setDirectionSwitched(boolean isDirectionSwitched) { this.isDirectionSwitched = isDirectionSwitched; }
}
