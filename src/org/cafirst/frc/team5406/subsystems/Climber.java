package org.cafirst.frc.team5406.subsystems;


import org.cafirst.frc.team5406.robot.Constants;
import com.ctre.CANTalon;
import org.cafirst.frc.team5406.subsystems.Subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Climber extends Subsystems{
	private CANTalon[] climberMotors;
	private int direction = 1;
	public boolean direction_switch = false;
	public Climber(){
		climberMotors = InitializeMotors(Constants.CLIMBER);
	}
	
	public void Climb(){
		climberMotors[0].enable();
		Climb(Constants.CLIMBER.target);
	}
	
	public void Climb(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		SmartDashboard.putNumber("Climber Current", climberMotors[0].getOutputCurrent());

		if (climberMotors[0].getOutputCurrent() > 30 && !direction_switch){
			direction *= -1;
			direction_switch = true;
		}
		climberMotors[0].set(direction*rpm);
		
	}
	
	public void DisplayCurrent(){
		DisplayCurrent(climberMotors);
	}
	
	public void StopClimb (){
		climberMotors[0].set(0);
	}
}
