package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.subsystems.Subsystems;


public class Intake extends Subsystems{
	private CANTalon[] intakeMotors;
	private long startTime = 0;
    public static DigitalInput gearSwitch = new DigitalInput(Constants.GEAR_SWITCH);
    private static DoubleSolenoid gearGripSolenoid = new DoubleSolenoid(Constants.GEAR_GRIP_FORWARD, Constants.GEAR_GRIP_REVERSE);;
    private static DoubleSolenoid gearLiftSolenoid = new DoubleSolenoid(Constants.GEAR_LIFT_FORWARD, Constants.GEAR_LIFT_REVERSE);;
	private boolean grip = false;
	private boolean gear_lift = false;
	public boolean gear_lifted = false;
	private int gear = 0;


	public Intake(){
		intakeMotors = InitializeMotors(Constants.INTAKE);
		intakeMotors[0].setVoltageRampRate(50);
	}
	
	public void IntakeBalls(){
		IntakeBalls(Constants.INTAKE.target);
	}
	
	public void IntakeBalls(double rpm){
		intakeMotors[0].enable();
		SmartDashboard.putNumber("Intake Current", intakeMotors[0].getOutputCurrent());
		/*if(System.nanoTime() - startTime < 1E9){
			intakeMotors[0].set(-0.5);
		}else{*/
			/*if (intakeMotors[0].getOutputCurrent() > 40){
				startTime = System.nanoTime();
				intakeMotors[0].set(-0.5);
			}else{*/
				intakeMotors[0].set(rpm);
			//}
		//}
	}
	
	public void OutputBalls(){
		IntakeBalls((double)(-0.5));
	}
	
	public void DisplayCurrent(){
		DisplayCurrent(intakeMotors);
	}
	
	public void autoGearLift(){
		SmartDashboard.putBoolean("Gear", haveGear());
		/*if(haveGear() && !gear_lifted){
			gear_lifted = true;
			liftGear();
		}*/
	}
	
	public boolean haveGear(){
		if(Constants.IS_PRACTICE_BOT){
			return !gearSwitch.get();
		}else{
			return gearSwitch.get();
		}
	}
	
	public void StopIntake (){
    	intakeMotors[0].disable();
	}
	
	public void gearUp(){
		gearLiftSolenoid.set(Constants.GEAR_LIFT_UP);
	}
	public void gearDown(){
		gearLiftSolenoid.set(Constants.GEAR_LIFT_DOWN);
	}
	public void gearGrip(){
		gearGripSolenoid.set(Constants.GEAR_GRIP_CLOSED);
	}
	public void gearRelease(){
		gearGripSolenoid.set(Constants.GEAR_GRIP_OPEN);
	}
	public void dropGear(boolean spin){
		grip = false;
		gearRelease();
		if(!haveGear() && spin){
			IntakeBalls(0.5);
		}
		System.out.println("dropGear");
    	new java.util.Timer().schedule( 
    	        new java.util.TimerTask() {
    	            @Override
    	            public void run() {
    	            	if(!grip){
    	            		gearDown();
    	            		//if(!haveGear() && spin){
    	            		if(spin){
    	            			IntakeBalls(1);
    	            		}
    	            		gear_lift = false;
    	            	}
    	            }
    	        }, 
    	        10 
    	);
    	
	}
	public void liftGear(){
		grip = true;
		gearGrip();
		System.out.println("liftGear");
    	new java.util.Timer().schedule( 
    	        new java.util.TimerTask() {
    	            @Override
    	            public void run() {
    	            	if(grip){
	    	            	gearUp();
	    	    			gear_lift = true;
    	            	}
    	            }
    	        }, 
    	        300 
    	);
		
	}
}
