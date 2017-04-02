package org.cafirst.frc.team5406.subsystems;

import java.util.Timer;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.vision.GripPipeline;
import org.cafirst.frc.team5406.vision.VisionListener;
import org.opencv.core.Point;


import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

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
	private Timer REVLimitTimer;
	private Timer turningTurretTimer;
	public boolean centeringInProgress = false;
	public boolean REVLimitSearching = false;
	private int autoTurretDirection = -1;
	private double distance;
	private boolean REVLimitFound = false;
	private int degreeTurnFinished = 0; //-1 = error; 0 = in progress; 1 = done
	private boolean degreeTurnInProgress = false;
	private int turretTimerRunning = 0;
	private double targetPosition;


	public Shooter(){

		shooterMotors = InitializeMotors(Constants.SHOOTER);
		indexerMotors = InitializeMotors(Constants.INDEXER);
		shooterMotors[0].setAllowableClosedLoopErr(20);
		indexerMotors[0].setAllowableClosedLoopErr(20);

		ballPumpMotors = InitializeMotors(Constants.BALL_PUMP);
		ballPumpMotors[0].setAllowableClosedLoopErr(20);


		turretMotors = InitializeMotors(Constants.TURRET);
		shooterSolenoid = new DoubleSolenoid(Constants.HOOD_FORWARD, Constants.HOOD_REVERSE);
		//shooterMotors[0].setVoltageRampRate(50);

		//turretMotors[0].enableBrakeMode(true);
		turretMotors[0].setCurrentLimit(10);
		turretMotors[0].EnableCurrentLimit(true);
		turretMotors[0].enableLimitSwitch(true, true);
		turretMotors[0].enableZeroSensorPositionOnForwardLimit(true);
		turretMotors[0].setAllowableClosedLoopErr(10);
		turretMotors[0].configPeakOutputVoltage(+5.0f, -5.0f);
		turretMotors[0].enableForwardSoftLimit(false);
		turretMotors[0].enableReverseSoftLimit(false);
		





		
		time_offset = System.nanoTime();
		
		//Instantiate items
		gripPipeline = new GripPipeline();
		listener = new VisionListener();
		axisCamera = CameraServer.getInstance().addAxisCamera(Constants.AXIS_IP);
		thread = new VisionThread(axisCamera, gripPipeline, listener);
		
		//Starts and runs thread
		thread.start();
		
	}
	
	public void getLimitSwitches(){
		SmartDashboard.putBoolean("FWD Limit", turretMotors[0].isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("REV Limit", turretMotors[0].isRevLimitSwitchClosed());
	}
	
	public void Shoot(){
		System.out.println("Shoot function");
		shooterMotors[0].enable();
		Shoot(getRPM());
	}
	
	

	
	public double getRPM(){
		double rpm;
 		Point topRight = listener.getTopRightPoint();
		if(distance < 0.5){
			getDistance();
			
		}
		
		if(Constants.IS_PRACTICE_BOT){
			//rpm = 298.36*distance + 3180.6;
			rpm = 0.6372*Math.pow(distance, 2)-51.553*distance+6111.7;
		}else{
			//rpm = 61.49*distance*distance-561.5*distance+6419;
			rpm = 0.1002*topRight.y*topRight.y-40.57*topRight.y+9003;
		}
		return rpm;
	}
	
	public void Shoot(double rpm){
		shooterMotors[0].changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterMotors[0].setF(0.5*(1023*600)/(rpm*4096));
    	shooterMotors[0].set(rpm);
    	displayEnc();
    	SmartDashboard.putNumber("TargetRPM", rpm);

	}
	
	public double getImageTop(){
		Point topRight = listener.getTopRightPoint();
		return topRight.y;
	}
	
	public void getDistance(){
 		Point topRight = listener.getTopRightPoint();
 		SmartDashboard.putNumber("topRight", topRight.y);
 		if(Constants.IS_PRACTICE_BOT){
 		//distance = 0.8779*(0.00007*Math.pow(topRight.y, 2)-0.0002*(topRight.y)+2.5956)+0.6104;
 		distance = (0.3092*topRight.y)-23.674;
 		}else{
 		distance = 0.92*(0.8371*(0.00007*Math.pow(topRight.y, 2)-0.0002*(topRight.y)+2.5956)+0.6104);
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
	
	
	public boolean moveTurret(double amount){
		turretMotors[0].changeControlMode(TalonControlMode.Position);
		boolean turn = true;
		boolean rev_limit = turretMotors[0].isRevLimitSwitchClosed();
		boolean fwd_limit = turretMotors[0].isFwdLimitSwitchClosed();
		//clockwise is negative ticks; 29422 ticks for full motion
		//clockwise hits reverse limit switch; counterclockwise hits forward limit switch
		if(fwd_limit){
			turn = (amount < 0);	
    		resetTurret();
    		autoTurretDirection = -1;
    		Constants.FWDLimit = turretMotors[0].getEncPosition();
    		//turretMotors[0].setForwardSoftLimit(-0.1);
    		//turretMotors[0].enableForwardSoftLimit(true);
    		//turretMotors[0].setReverseSoftLimit((-1*Constants.TURRET_ROTATION_TICKS/4096)+0.5);
    		//turretMotors[0].enableReverseSoftLimit(true);
		}else if (rev_limit){
			turn = (amount > 0);		
			Constants.REVLimit = turretMotors[0].getEncPosition();
    		autoTurretDirection = 1;
		}
		
		System.out.println("Turn: " + turn + "Turn Amount: " + amount + "FWDLimit: " + Constants.FWDLimit + ", REVLimit: " + Constants.REVLimit + "Current: " + turretMotors[0].getEncPosition());
		
		if(turn){
			turretPosition = turretMotors[0].getPosition()+amount;
			System.out.println("setPosition: " + turretPosition);
			turretMotors[0].set(turretPosition);
		}
		return rev_limit || fwd_limit;
	}
	
	public void turnTurret(double ticks, boolean overRideVision){
		if(turretTimerRunning == 0){
		System.out.println(System.nanoTime() + " TurnTurret");
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
	
	if(Math.abs(turnTicks) > 0.3){
		turnTicks = Math.signum(turnTicks)*0.3;
	}
	
	SmartDashboard.putNumber("turnTicks", turnTicks);
	moveTurret(turnTicks);
		}else{
			System.out.println("Turn Turret - Turret Timer Running");
		}
} 
	
	
	public void alignTurret(){
		if(!centeringInProgress){
		centeringTurret	 = new Timer();
		centeringTurret.schedule(new centerTurret(), 0L, 10L); //time in milliseconds
		centeringInProgress = true;
		}
	}
	
	public int turnTurretToDegree(double _degree){
		System.out.println("degreeTurnInProgress" + degreeTurnInProgress);
		if(!degreeTurnInProgress){
		turningTurretTimer	= new Timer();
		turningTurretTimer.schedule(new turnToDegree(_degree), 0L, 2L); //time in milliseconds
		degreeTurnInProgress = true;
		}
		return degreeTurnFinished;
	}
	
	
	public boolean findTurretREVLimit(){
		System.out.println("REVLimitSearching : " + REVLimitSearching);
		System.out.println("REVLimitFound : " + REVLimitFound);
		if(!REVLimitSearching){
		REVLimitTimer	 = new Timer();
		REVLimitTimer.schedule(new findREVLimit(), 0L, 2L); //time in milliseconds
		REVLimitSearching = true;
		}
		return REVLimitFound;
	}
	
	class centerTurret extends TimerTask {
		//Turret Timer 1
        public void run() {
        	if(turretTimerRunning ==0 || turretTimerRunning == 1){
        		turretTimerRunning =1;
    	double turnTicks = 0;
    	//need to make it go the other way if possible to reach target (edge case);
    	System.out.println(System.nanoTime() + " Boiler visible"  + listener.getBoilerVisible());
    	if(listener.getBoilerVisible()){
    		getDistance();
     		double center = (Constants.IMAGE_WIDTH/2)-Constants.IMAGE_WIDTH*((Math.tan(Constants.CAMERA_OFFSET/(distance*12)))*(180/Math.PI)/Constants.AXIS_FOV);
    		double centerOffset = center-Constants.centerX;
        	System.out.println(System.nanoTime() + " distance"  + distance);
        	System.out.println(System.nanoTime() + " center"  + center);
        	System.out.println(System.nanoTime() + " Constants.centerX"  + Constants.centerX);
        	System.out.println(System.nanoTime() + " centerOffset"  + centerOffset);

        	
    		SmartDashboard.putNumber("distance", distance);

     		SmartDashboard.putNumber("centerOffset", centerOffset);
    		double degreeOffset = Constants.AXIS_FOV*(centerOffset/Constants.IMAGE_WIDTH);
    		turnTicks = ((degreeOffset/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
    		if(Math.abs(centerOffset) > 20){
        	System.out.println(System.nanoTime() + " degreeOffset"  + degreeOffset);
        	System.out.println(System.nanoTime() + " turnTicks"  + turnTicks);
    		
    		SmartDashboard.putNumber("centerOffset", centerOffset);
    		SmartDashboard.putNumber("degreeOffset", degreeOffset);
    		SmartDashboard.putNumber("lastFrame", listener.getFrameCount());
    		turnTicks /= 10;
    		}else{
            	System.out.println(System.nanoTime() + "Final degreeOffset"  + degreeOffset);
            	System.out.println(System.nanoTime() + "Final turnTicks"  + turnTicks);

        		SmartDashboard.putNumber("centerOffset", centerOffset);
        		SmartDashboard.putNumber("degreeOffset", degreeOffset);
        		SmartDashboard.putNumber("lastFrame", listener.getFrameCount());

            	System.out.println(System.nanoTime() + " centeringDone");
    			turretTimerRunning = 0;
    			centeringInProgress = false;
    			centeringTurret.cancel();
    			centeringTurret.purge();
    		}
    		
    		
    	}else{
    		turnTicks = autoTurretDirection *0.2;
    	}
    	
    	if(Math.abs(turnTicks) > 0.15){
    		turnTicks = Math.signum(turnTicks)*0.15;
    	}
    	System.out.println("Ticks: " + turnTicks);
    	

    	
    	
    	//normally open switches, with signal connected to ground - default state is that signal pin is pulled high by RIO. A low signal represents a pressed button.
    	moveTurret(turnTicks);

    	
    	SmartDashboard.putNumber("CenterX", Constants.centerX);
    	SmartDashboard.putNumber("REVLimit_num", Constants.REVLimit);
    	SmartDashboard.putNumber("FWDLimit_num", Constants.FWDLimit);

        
        }else{
        	System.out.println("Center Turret - Turret Timer Running");
        }
	}
        
    }
	
	
	
	class findREVLimit extends TimerTask {
		//Turret Timer 2
    	boolean turn = true;
    	double turnTicks = 0;
    	
    	public findREVLimit(){
    		
    	}

	    public void run() {
	    	if(turretTimerRunning ==0 || turretTimerRunning ==2){
	    		turretTimerRunning = 2;
			if (!REVLimitFound){
				 turnTicks = 0.1;
			}
	    	System.out.println("turretPosition" + turretMotors[0].getPosition() + "REVLimit" + Constants.REVLimit);
	    	moveTurret(turnTicks);

	    	if(Constants.FWDLimit < Integer.MAX_VALUE){
	    		resetTurret();
	    		Constants.FWDLimit = turretMotors[0].getEncPosition();
	    		turretTimerRunning = 0;
	    		REVLimitFound = true;
	    		REVLimitTimer.cancel();
	    		REVLimitTimer.purge();
	    	}
	        }
	    }
	}
	
	
	class turnToDegree extends TimerTask {
    	private double turnTicks = 0;
    	
    	private double turnDegrees = 0;
    	double targetPosition;
    	
    	public turnToDegree(double _turnDegrees){
    		turnDegrees = _turnDegrees;
    		degreeTurnFinished = 0;
    	}
    	
    	

	    public void run() {
			System.out.println("turretTimerRunning" + turretTimerRunning);
			
	    	if(turretTimerRunning ==0||turretTimerRunning ==3){
	    		turretTimerRunning = 3;
			targetPosition =   (Constants.FWDLimit +(turnDegrees/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
	    	System.out.println("targetPosition: " + targetPosition + ", current: " + turretMotors[0].getPosition());
	    	
			if (REVLimitFound){
		    	SmartDashboard.putNumber("targetPosition", targetPosition);
		    	SmartDashboard.putNumber("turretPosition", turretMotors[0].getPosition());
				if(turretMotors[0].getPosition() > targetPosition - 0.1 && turretMotors[0].getPosition() < targetPosition + 0.1){
			    	System.out.println("Cancelling Timer 1");
					degreeTurnFinished = 1;
					turretTimerRunning = 0;
		    		turningTurretTimer.cancel();
		    		turningTurretTimer.purge();
				}else{
				turnTicks = -0.1;
		    	//clockwise is negative ticks; 29422 ticks for full motion
		    	if(turretMotors[0].isRevLimitSwitchClosed()){
			    	System.out.println("Cancelling Timer 2");
		    		degreeTurnFinished = -1;
		    		turretTimerRunning = 0;
		    		turningTurretTimer.cancel();
		    		turningTurretTimer.purge();
		    	}
		    	
		    	moveTurret(turnTicks);
				}
			
			}
	    	}else{
	    		System.out.println("FindREVLimit - Turret Timer Running");
	    	}
	    	
	        }
	}
	
	
	/*class turnToDegreeSpeed extends TimerTask {
    	private boolean turn = true;
    	private double turnRPM = 0;
    	
    	private double turnDegrees = 0;
    	double targetPosition;
    	
    	public turnToDegreeSpeed(double _turnDegrees){
    		turnDegrees = _turnDegrees;
    		degreeTurnFinished = 0;
    		turretMotors[0].changeControlMode(TalonControlMode.Speed);
    	}
    	
    	

	    public void run() {
			targetPosition =   (Constants.REVLimit +(turnDegrees/Constants.TURRET_ROTATION_DEG)*Constants.TURRET_ROTATION_TICKS)/4096;
	    	System.out.println("targetPosition: " + targetPosition + ", current: " + turretMotors[0].getPosition());
	    	
			if (REVLimitFound){
		    	SmartDashboard.putNumber("targetPosition", targetPosition);
		    	SmartDashboard.putNumber("turretPosition", turretMotors[0].getPosition());
				if(turretMotors[0].getPosition() > targetPosition - 0.1 && turretMotors[0].getPosition() < targetPosition + 0.1){
					degreeTurnFinished = 1;
		    		turretMotors[0].set(0);
		    		turretMotors[0].changeControlMode(TalonControlMode.Position);
		    		turretMotors[0].set(turretMotors[0].getEncPosition());
		    		turningTurretTimer.cancel();
		    		turningTurretTimer.purge();
				}else{
				turnRPM = -10;
				boolean ccw_limit = !turretLimitSwitchCCW.get();
		    	boolean cw_limit = !turretLimitSwitchCW.get();
		    	//clockwise is negative ticks; 29422 ticks for full motion
		    	if(ccw_limit){
		    		turn = (turnRPM < 0);	
		    		autoTurretDirection = -1;
		    		Constants.REVLimit = turretMotors[0].getEncPosition();
		    	}else if (cw_limit){
		    		turn = (turnRPM > 0);		
		    		Constants.CWLimit = turretMotors[0].getEncPosition();
		    		autoTurretDirection = 1;
		    	}
		    	
		    	if(turn){
		    		turretMotors[0].set(turnRPM);
		    	}else{
		    		degreeTurnFinished = -1;
		    	}
				}
			
			}

	    	
	        }
	}*/
	
	public void stopTimer(){
		if(centeringTurret !=null){
		centeringTurret.cancel();
		}
		if(REVLimitTimer !=null){
		REVLimitTimer.cancel();
		}
		if(turningTurretTimer !=null){
		turningTurretTimer.cancel();
		}
		turretTimerRunning = 0;
	}
    
    
	/*public double autoTurret(){
		turretMotors[0].set(20);
		return turretMotors[0].getEncPosition();
	}*/
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
				
				if(turnPos < Constants.REVLimit){
					turnRadians = 2*Math.PI + turnPos;
				}else if (turnPos > Constants.CWLimit){
					turnRadians = turnPos-2*Math.PI;
				}
				
				if(turnPos < Constants.REVLimit || turnPos > Constants.CWLimit){
					StopTurret();
				}else{
					//turretMotors[0].set((turnPos-indexerMotors[0].getEncPosition())/4096);
				}*/
	}
	
	/*public void lockTurret(boolean lock){
		//turretMotors[0].enableBrakeMode(lock);
	}*/
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
