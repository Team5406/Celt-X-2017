package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.vision.GripPipeline;
import org.cafirst.frc.team5406.vision.MyThread;

import com.ctre.CANTalon;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.vision.GripPipeline;
import org.cafirst.frc.team5406.vision.MyThread;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class Shooter extends Subsystems{
	private CANTalon[] shooterMotors;
	private CANTalon[] indexerMotors;
	private CANTalon[] ballPumpMotors;
	private CANTalon[] turretMotors;
    private DoubleSolenoid shooterSolenoid;
    DigitalInput turretLimitSwitchCW;
    DigitalInput turretLimitSwitchCCW;
	//Filters Image, You must GripPipeline.java into the new Robot code for it to work
	GripPipeline gripPipeline;
	//Camera being used
	AxisCamera axisCamera;
	//Runs Vision Scanning
	MyThread thread;
	public int rpm = 6000;
	private double turretPosition = 0;
	private long time_offset =0;
	public Shooter(){

		shooterMotors = InitializeMotors(Constants.SHOOTER);
		indexerMotors = InitializeMotors(Constants.INDEXER);
		ballPumpMotors = InitializeMotors(Constants.BALL_PUMP);
		turretLimitSwitchCW = new DigitalInput(Constants.CW_LIMIT_SWITCH);
		turretLimitSwitchCCW = new DigitalInput(Constants.CCW_LIMIT_SWITCH);

		turretMotors = InitializeMotors(Constants.TURRET);
		shooterSolenoid = new DoubleSolenoid(Constants.HOOD_FORWARD, Constants.HOOD_REVERSE);
		//shooterMotors[0].setVoltageRampRate(50);

		//turretMotors[0].enableBrakeMode(true);
		
		time_offset = System.nanoTime();
		
		//Instantiate items
		gripPipeline = new GripPipeline();
		axisCamera = CameraServer.getInstance().addAxisCamera(Constants.AXIS_IP);
		thread = new MyThread(gripPipeline, axisCamera);
		
		//Starts and runs thread
		thread.start();
		
		SmartDashboard.putBoolean("CCW Limit", false);
		SmartDashboard.putBoolean("CW Limit", false);
	}
	
	public void Shoot(){
		System.out.println("Shoot function");
		shooterMotors[0].enable();
		Shoot(Constants.SHOOTER.target);
	}
	
	public void Shoot(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		//shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterMotors[0].setF(0.015);
    	//shooterMotors[0].set(7700);
    	shooterMotors[0].set(7000);
    	//shooterMotors[0].set(0.2);
   	

	}
	public void BallPump(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		ballPumpMotors[0].set(rpm);
	}
	
	public void Indexer(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		indexerMotors[0].set(rpm);
	}
	
	public void turnTurret(double ticks){
		System.out.println(System.nanoTime() + " TurnTurret");
	boolean turn = true;
	
	double turnTicks = ticks;
	SmartDashboard.putNumber("turnTicks", turnTicks);
	
	//normally open switches, with signal connected to ground - default state is that signal pin is pulled high by RIO. A low signal represents a pressed button.
	boolean ccw_limit = !turretLimitSwitchCCW.get();
	boolean cw_limit = !turretLimitSwitchCW.get();
	//clockwise is negative ticks; 29422 ticks for full motion
	//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
	if(ccw_limit){
		turn = (turnTicks < 0);	
		Constants.CCWLimit = turretMotors[0].getEncPosition();
	}else if (cw_limit){
		turn = (turnTicks > 0);		
		Constants.CWLimit = turretMotors[0].getEncPosition();
	}
	
	if(turn){
		turretPosition = turretMotors[0].getPosition()+ticks/5;
		turretMotors[0].set(turretPosition);
		
	}
	
	SmartDashboard.putNumber("CCWLimit_num", Constants.CCWLimit);
	SmartDashboard.putNumber("CWLimit_num", Constants.CWLimit);

	SmartDashboard.putBoolean("CCW Limit", ccw_limit);
	SmartDashboard.putBoolean("CW Limit", cw_limit);
} 

	
	public void positionTurret(){
				
				/* 4096 ticks = 2pi rad
				 * frame width represents pi/3 rad; half width, pi/6 rad;
				 * turret turns 3pi/2 rad; pi/2 rad unreachable.
				*/
		
				double turnRadians = 0;
				double encTicksPerRad = 4096/(2*Math.PI);
				double turnTicks = 4096/(2*Math.PI);
				double turnPos;
				
				turnRadians = Math.atan(((Constants.centerX - Constants.IMAGE_CENTER)*Math.tan(Constants.CAMERA_FIELD_HALF))/Constants.IMAGE_CENTER);
				turnTicks = turnRadians * encTicksPerRad;
				turnPos = turretMotors[0].getEncPosition() +turnTicks;
				
				if(turnPos < Constants.CCWLimit){
					turnRadians = 2*Math.PI + turnPos;
				}else if (turnPos > Constants.CWLimit){
					turnRadians = turnPos-2*Math.PI;
				}
				
				if(turnPos < Constants.CCWLimit || turnPos > Constants.CWLimit){
					StopTurret();
				}else{
					//turretMotors[0].set((turnPos-indexerMotors[0].getEncPosition())/4096);
				}
	}
	
	public void lockTurret(boolean lock){
		turretMotors[0].enableBrakeMode(lock);
	}
	public void DisplayCurrent(){
		DisplayCurrent(indexerMotors);
	}
	
	public void StopShoot (){
		shooterMotors[0].set(0);
		shooterMotors[0].setF(0);
		shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.PercentVbus);

	}
	public void StopTurret (){
		//turretMotors[0].set(0);
	}
	public void StopIndexer (){
		indexerMotors[0].set(0);
	}
	public void StopBallPump (){
		ballPumpMotors[0].set(0);
	}
	
	public void displayEnc(){
		SmartDashboard.putNumber("TurretEncPos", turretMotors[0].getEncPosition());
		SmartDashboard.putNumber("TurretPos", turretMotors[0].getPosition());
		SmartDashboard.putNumber("TurretPositionSet", turretPosition);
    	SmartDashboard.putNumber("RPM", shooterMotors[0].getSpeed());
    	double motorOutputA = shooterMotors[0].getOutputVoltage() / shooterMotors[0].getBusVoltage();
    	double motorOutputB = shooterMotors[1].getOutputVoltage() / shooterMotors[1].getBusVoltage();

	    SmartDashboard.putNumber("%VBusA", motorOutputA);
	    SmartDashboard.putNumber("%VBusB", motorOutputB);
	    SmartDashboard.putNumber("IA", shooterMotors[0].getOutputCurrent());
	    SmartDashboard.putNumber("IB", shooterMotors[1].getOutputCurrent());
	    SmartDashboard.putNumber("Error", shooterMotors[0].getClosedLoopError());

        

            

        	System.out.println("Time: " + Math.round((System.nanoTime()-time_offset)/1e6) + "ms; outA: " +  motorOutputA + "; outB: " + motorOutputB + " spd: " + shooterMotors[0].getSpeed() + "; err: " +shooterMotors[0].getClosedLoopError());
        

	    
	}
	
	public void hoodUp(){
		shooterSolenoid.set(Constants.HOOD_UP);
	}
	public void hoodDown(){
		shooterSolenoid.set(Constants.HOOD_DOWN);
	}
}
