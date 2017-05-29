package org.cafirst.frc.team5406.subsystem;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.util.Constants;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends Subsystem {
	
	private CANTalon intakeMotor;
	
    private DigitalInput gearSwitch;
    @SuppressWarnings("unused")
	private DigitalOutput gearLight;
    private DoubleSolenoid gearGripSolenoid;
    private DoubleSolenoid gearLiftSolenoid;
	
    private boolean isGripping = false;
    
	public Intake() 
	{
		super(Constants.intake);
		
		intakeMotor = initializeMotors(Constants.INTAKE_ID, true, 0);
		intakeMotor.setVoltageRampRate(50);
		intakeMotor.setAllowableClosedLoopErr(10);
		intakeMotor.setCurrentLimit(20);
		
		gearSwitch = new DigitalInput(Constants.GEAR_SWITCH);
		gearLight = new DigitalOutput(Constants.GEAR_LIGHT);
		gearGripSolenoid = new DoubleSolenoid(Constants.GEAR_GRIP_FORWARD, Constants.GEAR_GRIP_REVERSE);
		gearLiftSolenoid = new DoubleSolenoid(Constants.GEAR_LIFT_FORWARD, Constants.GEAR_LIFT_REVERSE);
	}
	
	/**
	 * Spins the intake at the desired rpm
	 * @param rpm The desired rpm
	 */
	public void intakeBalls(double rpm)
	{
		intakeMotor.enable();
		intakeMotor.set(rpm);
	}
	
	/**Spins the intake at the target rpm*/
	public void intakeBalls() { intakeBalls(Constants.intake.getTarget()); }

	/**Spins the intake in the reverse direction to output balls*/
	public void outputBalls() { intakeBalls(-0.5); }
	
	/**Disable the intake*/
	public void stopIntake() { intakeMotor.disable(); }
	
	/**
	 * Checks to see if robot has a gear
	 * @return True if robot has a gear
	 */
	public boolean isGearHad() { return gearSwitch.get(); }
	
	/**Raises gear intake*/
	public void gearUp(){ gearLiftSolenoid.set(Constants.GEAR_LIFT_UP); }
	
	/**Lowers gear intake*/
	public void gearDown(){ gearLiftSolenoid.set(Constants.GEAR_LIFT_DOWN); }
	
	/**Closes gear intake arms*/
	public void gearGrip(){ gearGripSolenoid.set(Constants.GEAR_GRIP_CLOSED); }
	
	/**Opens gear intake arms*/
	public void gearRelease(){ gearGripSolenoid.set(Constants.GEAR_GRIP_OPEN); }
	
	/**
	 * Drops gear intake and opens the arms
	 * @param spin True if you desire the gear intake to spin
	 */
	public void dropGear(boolean spin){
		isGripping = false;
		gearRelease();
		if(!isGearHad() && spin)
			intakeBalls(0.5);
		
    	new Timer().schedule( 
    	        new TimerTask() {
    	            @Override
    	            public void run() {
    	            	if(!isGripping)
    	            	{
    	            		gearDown();
    	            		if(spin)
    	            			intakeBalls(1);
    	            	}
    	            }
    	        }, 
    	        10 
    	);
    	
	}
	
	/**Closes arms and raises gear intake*/
	public void liftGear(){
		isGripping = true;
		gearGrip();
		System.out.println("liftGear");
    	new Timer().schedule( 
    	        new TimerTask() {
    	            @Override
    	            public void run() {
    	            	if(isGripping){
	    	            	gearUp();
    	            	}
    	            }
    	        }, 
    	        300 
    	);
		
	}
}
