package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.subsystems.Subsystems;


public class Intake extends Subsystems{
	private CANTalon[] intakeMotors;
	private long startTime = 0;
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
		if(System.nanoTime() - startTime < 1E9){
			intakeMotors[0].set(0.5);
		}else{
			if (intakeMotors[0].getOutputCurrent() > 40){
				startTime = System.nanoTime();
				intakeMotors[0].set(0.5);
			}else{
				intakeMotors[0].set(rpm);
			}
		}
	}
	
	public void OutputBalls(){
		IntakeBalls((double)(-0.5));
	}
	
	public void DisplayCurrent(){
		DisplayCurrent(intakeMotors);
	}
	
	public void StopIntake (){
    	intakeMotors[0].disable();
	}
}
