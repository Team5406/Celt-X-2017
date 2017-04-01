package org.cafirst.frc.team5406.subsystems;

import java.util.Timer;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.vision.GripPipeline;
import org.cafirst.frc.team5406.vision.VisionListener;
import org.opencv.core.Point;


import com.ctre.CANTalon;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import java.util.Timer;
import java.util.TimerTask;

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
	VisionListener listener;
	VisionThread thread;
	public int rpm = 5700;
	private double turretPosition = 0;
	private long time_offset =0;
	private long lastFrame = -1;
	private Timer centeringTurret;
	private Timer CCWLimitTimer;
	private Timer turningTurretTimer;
	public boolean centeringInProgress = false;
	public boolean CCWLimitSearching = false;
	private int autoTurretDirection = -1;
	private double distance;
	private boolean CCWLimitFound = false;
	private int degreeTurnFinished = 0; //-1 = error; 0 = in progress; 1 = done
	private boolean degreeTurnInProgress = false;


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
		listener = new VisionListener();
		axisCamera = CameraServer.getInstance().addAxisCamera(Constants.AXIS_IP);
		thread = new VisionThread(axisCamera, gripPipeline, listener);
		
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
		if(distance < 0.5){
			getDistance();
		}
		if(Constants.IS_PRACTICE_BOT){
			rpm = 298.36*distance + 3180.6;
		}else{
			rpm = 242*distance + 3400;
		}
		//}
		shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterMotors[0].setF(0.5*(1023*600)/(rpm*4096));
    	shooterMotors[0].set(rpm);
    	displayEnc();
    	
    	SmartDashboard.putNumber("TargetRPM", rpm);

	}
	
	public void getDistance(){
 		Point topRight = listener.getTopRightPoint();
 		if(Constants.IS_PRACTICE_BOT){
 		distance = 0.8779*(0.00007*Math.pow(topRight.y, 2)-0.0002*(topRight.y)+2.5956)+0.6104;
 		}else{
 		distance = 0.8371*(0.00007*Math.pow(topRight.y, 2)-0.0002*(topRight.y)+2.5956)+0.6104;
 		}
	}
	
	
	public void BallPump(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		ballPumpMotors[0].set(rpm);
	}
	
	public void Indexer(double rpm){
		//need to confirm that motor is turning - otherwise try reverse direction due to ratcheting
		indexerMotors[0].changeControlMode(CANTalon.TalonControlMode.Speed);
		indexerMotors[0].set(rpm);
	}
	
	public void turnTurret(double ticks, boolean overRideVision){
		System.out.println(System.nanoTime() + " TurnTurret");
	boolean turn = true;
	
	double turnTicks;
	
	
	if(listener.getBoilerVisible() && !overRideVision){
		
		double centerOffset = (Constants.IMAGE_WIDTH/2)-Constants.centerX;
		double degreeOffset = Constants.AXIS_FOV*(centerOffset/Constants.IMAGE_WIDTH);
		turnTicks = ((degreeOffset/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
		
		
		SmartDashboard.putNumber("centerOffset", centerOffset);
		SmartDashboard.putNumber("degreeOffset", degreeOffset);
		SmartDashboard.putNumber("lastFrame", listener.getFrameCount());

		/*
		if(lastFrame == listener.getFrameCount()){
			turnTicks /= 5;  //corrects for difference between 15Hz camera and 50Hz periodic function
		}else{
			if(turnTicks > 0.2){
				turnTicks /= 3;
			}
			lastFrame = listener.getFrameCount();
		}*/
		
		turnTicks /= 5;
		
		
	}else{
		turnTicks = ticks/5;
	}
	
	if(Math.abs(turnTicks) > 0.15){
		turnTicks = Math.signum(turnTicks)*0.15;
	}
	
	SmartDashboard.putNumber("turnTicks", turnTicks);

	
	
	//normally open switches, with signal connected to ground - default state is that signal pin is pulled high by RIO. A low signal represents a pressed button.
	boolean ccw_limit = !turretLimitSwitchCCW.get();
	boolean cw_limit = !turretLimitSwitchCW.get();
	//clockwise is negative ticks; 29422 ticks for full motion
	if(ccw_limit){
		turn = (turnTicks < 0);	
		Constants.CCWLimit = turretMotors[0].getEncPosition();
	}else if (cw_limit){
		turn = (turnTicks > 0);		
		Constants.CWLimit = turretMotors[0].getEncPosition();
	}
	
	if(turn){
		turretPosition = turretMotors[0].getPosition()+turnTicks;
		turretMotors[0].set(turretPosition);
	}
	
	
	SmartDashboard.putNumber("CenterX", Constants.centerX);
	SmartDashboard.putNumber("CCWLimit_num", Constants.CCWLimit);
	SmartDashboard.putNumber("CWLimit_num", Constants.CWLimit);

	SmartDashboard.putBoolean("CCW Limit", ccw_limit);
	SmartDashboard.putBoolean("CW Limit", cw_limit);
} 
	
	
	public void alignTurret(){
		if(!centeringInProgress){
		centeringTurret	 = new Timer();
		centeringTurret.schedule(new centerTurret(), 0L, 10L); //time in milliseconds
		centeringInProgress = true;
		}
	}
	
	public int turnTurretToDegree(double _degree){
		if(!degreeTurnInProgress){
		turningTurretTimer	 = new Timer();
		turningTurretTimer.schedule(new turnToDegree(_degree), 0L, 2L); //time in milliseconds
		degreeTurnInProgress = true;
		}
		return degreeTurnFinished;
	}
	
	public boolean findTurretCCWLimit(){
		if(!CCWLimitSearching){
		CCWLimitTimer	 = new Timer();
		CCWLimitTimer.schedule(new findCCWLimit(), 0L, 2L); //time in milliseconds
		CCWLimitSearching = true;
		}
		return CCWLimitFound;
	}
	
	class centerTurret extends TimerTask {
        public void run() {
    	boolean turn = true;
    	double turnTicks = 0;
    	//need to make it go the other way if possible to reach target (edge case);
    	
    	if(listener.getBoilerVisible()){
    		getDistance();
     		double center = (Constants.IMAGE_WIDTH/2)-20-Constants.IMAGE_WIDTH*((Math.tan(Constants.CAMERA_OFFSET/(distance*12)))*(180/Math.PI)/Constants.AXIS_FOV);
    		double centerOffset = center-Constants.centerX;
        	
    		SmartDashboard.putNumber("distance", distance);

    		
    		if(Math.abs(centerOffset) > 4){
    		double degreeOffset = Constants.AXIS_FOV*(centerOffset/Constants.IMAGE_WIDTH);
    		turnTicks = ((degreeOffset/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
    		
    		
    		SmartDashboard.putNumber("centerOffset", centerOffset);
    		SmartDashboard.putNumber("degreeOffset", degreeOffset);
    		SmartDashboard.putNumber("lastFrame", listener.getFrameCount());
    		turnTicks /= 10;
    		}else{
    			centeringTurret.cancel();
    			centeringTurret.purge();
    			centeringInProgress = false;
    		}
    		
    		
    	}else{
    		turnTicks = autoTurretDirection *0.03;
    	}
    	
    	if(Math.abs(turnTicks) > 0.15){
    		turnTicks = Math.signum(turnTicks)*0.15;
    	}
    	
    	SmartDashboard.putNumber("turnTicks", turnTicks);

    	
    	
    	//normally open switches, with signal connected to ground - default state is that signal pin is pulled high by RIO. A low signal represents a pressed button.
    	boolean ccw_limit = !turretLimitSwitchCCW.get();
    	boolean cw_limit = !turretLimitSwitchCW.get();
    	//clockwise is negative ticks; 29422 ticks for full motion
    	if(ccw_limit){
    		turn = (turnTicks < 0);	
    		autoTurretDirection = -1;
    		Constants.CCWLimit = turretMotors[0].getEncPosition();
    	}else if (cw_limit){
    		turn = (turnTicks > 0);		
    		Constants.CWLimit = turretMotors[0].getEncPosition();
    		autoTurretDirection = 1;
    	}
    	
    	if(turn){
    		turretPosition = turretMotors[0].getPosition()+turnTicks;
        	SmartDashboard.putNumber("turretSetPosition", turretPosition);
    		turretMotors[0].set(turretPosition);
    	}
    	

    	
    	
    	SmartDashboard.putNumber("CenterX", Constants.centerX);
    	SmartDashboard.putNumber("CCWLimit_num", Constants.CCWLimit);
    	SmartDashboard.putNumber("CWLimit_num", Constants.CWLimit);

    	SmartDashboard.putBoolean("CCW Limit", ccw_limit);
    	SmartDashboard.putBoolean("CW Limit", cw_limit);
        }
        
        
    }
	
	
	
	class findCCWLimit extends TimerTask {
    	boolean turn = true;
    	double turnTicks = 0;
    	
    	public findCCWLimit(){
    		
    	}

	    public void run() {
			if (!CCWLimitFound){
				 turnTicks = 0.1;
			}
	    	SmartDashboard.putNumber("turretPosition", turretMotors[0].getPosition());
	    	SmartDashboard.putBoolean("CCWLimitFound", CCWLimitFound);
	    	SmartDashboard.putNumber("CCWLimit", Constants.CCWLimit);

	    	boolean ccw_limit = !turretLimitSwitchCCW.get();
	    	boolean cw_limit = !turretLimitSwitchCW.get();
	    	//clockwise is negative ticks; 29422 ticks for full motion
	    	if(ccw_limit){
	    		resetTurret();
	    		turn = (turnTicks < 0);	
	    		autoTurretDirection = -1;
	    		Constants.CCWLimit = turretMotors[0].getEncPosition();
	    		CCWLimitFound = true;
	    		CCWLimitTimer.cancel();
	    		CCWLimitTimer.purge();

	    	}else if (cw_limit){ //shouldn't happen
	    		turn = (turnTicks > 0);		
	    		Constants.CWLimit = turretMotors[0].getEncPosition();
	    		autoTurretDirection = 1;
	    	}
	    	
	    	if(turn){
	    		turretPosition = turretMotors[0].getPosition()+turnTicks;
	    		turretMotors[0].set(turretPosition);
	    	}
	        }
	}
	
	
	class turnToDegree extends TimerTask {
    	private boolean turn = true;
    	private double turnTicks = 0;
    	
    	private double turnDegrees = 0;
    	double targetPosition;
    	
    	public turnToDegree(double _turnDegrees){
    		turnDegrees = _turnDegrees;
    		degreeTurnFinished = 0;
    	}
    	
    	

	    public void run() {
			targetPosition =   (Constants.CCWLimit +(turnDegrees/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
	    	System.out.println("targetPosition: " + targetPosition + ", current: " + turretMotors[0].getPosition());
	    	
			if (CCWLimitFound){
		    	SmartDashboard.putNumber("targetPosition", targetPosition);
		    	SmartDashboard.putNumber("turretPosition", turretMotors[0].getPosition());
				if(turretMotors[0].getPosition() > targetPosition - 0.1 && turretMotors[0].getPosition() < targetPosition + 0.1){
					degreeTurnFinished = 1;
		    		turningTurretTimer.cancel();
		    		turningTurretTimer.purge();
				}else{
				turnTicks = -0.1;
				boolean ccw_limit = !turretLimitSwitchCCW.get();
		    	boolean cw_limit = !turretLimitSwitchCW.get();
		    	//clockwise is negative ticks; 29422 ticks for full motion
		    	if(ccw_limit){
		    		turn = (turnTicks < 0);	
		    		autoTurretDirection = -1;
		    		Constants.CCWLimit = turretMotors[0].getEncPosition();
		    	}else if (cw_limit){
		    		turn = (turnTicks > 0);		
		    		Constants.CWLimit = turretMotors[0].getEncPosition();
		    		autoTurretDirection = 1;
		    	}
		    	
		    	if(turn){
		    		turretPosition = turretMotors[0].getPosition()+turnTicks;
		    		turretMotors[0].set(turretPosition);
		    	}else{
		    		degreeTurnFinished = -1;
		    	}
				}
			
			}

	    	
	        }
	}
	
	public void stopTimer(){
		centeringTurret.cancel();
		CCWLimitTimer.cancel();
		turningTurretTimer.cancel();
	}
    
    
	public double autoTurret(){
		turretMotors[0].set(20);
		return turretMotors[0].getEncPosition();
	}
	public void resetTurret(){
		turretMotors[0].setEncPosition(0);
		turretMotors[0].setPosition(0);
	}

	
	public void positionTurret(){
		
		
				
				/* 4096 ticks = 2pi rad
				 * frame width represents pi/3 rad; half width, pi/6 rad;
				 * turret turns 3pi/2 rad; pi/2 rad unreachable.
				*/
		
				/*double turnRadians = 0;
				double encTicksPerRad = 4096/(2*Math.PI);
				double turnTicks = 4096/(2*Math.PI);
				double turnPos;
				
				Point bottomLeft = listener.getBottomLeftPoint();
				Point topRight = listener.getTopRightPoint();
				double center = (bottomLeft.x + topRight.x)/2;
				
				turnRadians = Math.atan(((center - Constants.IMAGE_CENTER)*Math.tan(Constants.CAMERA_FIELD_HALF))/Constants.IMAGE_CENTER);
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
				}*/
	}
	
	public void lockTurret(boolean lock){
		turretMotors[0].enableBrakeMode(lock);
	}
	public void DisplayCurrent(){
		DisplayCurrent(indexerMotors);
	}
	
	public void StopShoot (){
		shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		shooterMotors[0].set(0);
		//shooterMotors[0].setF(0);

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
