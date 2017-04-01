package org.cafirst.frc.team5406.util;
import org.cafirst.frc.team5406.robot.Constants;

public class Util {
	public Util(){
		
	}
	
	public double applyDeadband(double value){
		return applyDeadband(value, Constants.xboxControllerDeadband);
	}
	
	public double applyDeadband(double value, double deadband){
		if ((-1*deadband) < value && value < deadband){
			return 0;
		}else{
			return value;
		}
	}
}
