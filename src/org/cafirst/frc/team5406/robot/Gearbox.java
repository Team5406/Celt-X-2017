package org.cafirst.frc.team5406.robot;
import com.ctre.CANTalon;

public class Gearbox {
	int encoder_ratio;
	public int stall_current;
	public double free_current;
	public int free_speed;
	public int gearing;
	public double target;
	public CANTalon.TalonControlMode mode;
	public boolean reverse_encoder;
	public double PID_F;
	public double PID_P;
	public double PID_I;
	public double PID_D;
	public Motor[] motors;
	    
 
	public Gearbox(){
	}
	
}
