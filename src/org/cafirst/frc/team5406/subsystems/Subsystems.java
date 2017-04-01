package org.cafirst.frc.team5406.subsystems;
import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon.FeedbackDevice;


import org.cafirst.frc.team5406.robot.Motor;
import org.cafirst.frc.team5406.robot.Gearbox;


public class Subsystems {
	public Subsystems(){
		
	}
	
	public CANTalon[] InitializeMotors(Gearbox subsystem_gearbox){
		CANTalon[] motors = new CANTalon[subsystem_gearbox.motors.length];
		for (int i = 0; i<subsystem_gearbox.motors.length; i++){
			Motor motor = subsystem_gearbox.motors[i];
			
			motors[i] = new CANTalon(motor.id);
			motors[i].configNominalOutputVoltage(+0.0f, -0.0f);
			motors[i].configPeakOutputVoltage(+12.0f, -12.0f);
			
			if(!motor.master){
				motors[i].changeControlMode(TalonControlMode.Follower);
				motors[i].set(motor.follow); //pass ID of primary motor
			}else{
				motors[i].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
				motors[i].reverseSensor(subsystem_gearbox.reverse_encoder);
				motors[i].setProfile(0);
				motors[i].setF(subsystem_gearbox.PID_F);
				motors[i].setP(subsystem_gearbox.PID_P);
				motors[i].setI(subsystem_gearbox.PID_I); 
				motors[i].setD(subsystem_gearbox.PID_D);
				motors[i].changeControlMode(subsystem_gearbox.mode);
				motors[i].setInverted(subsystem_gearbox.invert);

			}
		}
		return motors;
	}
	
	public void DisplayCurrent(CANTalon[] motors){
		for (CANTalon motor : motors){
			SmartDashboard.putNumber("Motor Current (" + motor.getDescription() +")", motor.getOutputCurrent());
		}
	}
}
