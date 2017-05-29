package org.cafirst.frc.team5406.subsystem;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.util.Constants;
import org.cafirst.frc.team5406.vision.GripPipeline;
import org.cafirst.frc.team5406.vision.VisionListener;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Shooter extends Subsystem{

	private CANTalon shooterMasterMotor;
	@SuppressWarnings("unused")
	private CANTalon shooterSlaveMotor;
	private CANTalon indexerMotor;
	private CANTalon ballPumpMotor;
	private CANTalon turretMotor;
	
	private AxisCamera visionCamera;
	private GripPipeline visionPipeline;
	private VisionThread visionThread;
	private VisionListener visionListener;
	
	/**0 = No Timer Running; 1 = Vision Timer Running; 2 = Limit Switch Timer Running*/
	private int turretTimerRunning = 0; 
	
	private Timer visionCenteringTimer;
	private boolean isVisionCentering = false;
	private Timer findLimitSwitchTimer;
	private boolean isFindingLimitSwitch = false;
	private boolean isLimitSwitchFound = false;
	
	private long lastFrame = -1;
	
	private int shooterRPMOffset = 0;
	
	private int autoTurretDirection = -1;
	private double turretPosition = 0;
	
	private double turretTarget;
	
	private double degreeTarget;
	private boolean isDegreeTurning;
	
	private double cameraOffset;
	private boolean isVisionAvailable = false;
	
	public Shooter() 
	{
		super(Constants.shooter);
		
		shooterMasterMotor = initializeMotors(Constants.SHOOTER_MASTER_ID, true, 0);
		shooterMasterMotor.setAllowableClosedLoopErr(20);
		shooterSlaveMotor = initializeMotors(Constants.SHOOTER_SLAVE_ID, false, Constants.SHOOTER_MASTER_ID);
		
		indexerMotor = initializeMotors(Constants.INDEXER_ID, true, 0);
		indexerMotor.setAllowableClosedLoopErr(20);
		
		ballPumpMotor = initializeMotors(Constants.BALL_PUMP_ID, true, 0);
		ballPumpMotor.setAllowableClosedLoopErr(20);
		ballPumpMotor.setVoltageRampRate(80);
		
		turretMotor = initializeMotors(Constants.TURRET_ID, true, 0);
		turretMotor.setCurrentLimit(10);
		turretMotor.EnableCurrentLimit(true);
		turretMotor.enableLimitSwitch(true, true);
		turretMotor.enableZeroSensorPositionOnForwardLimit(true);
		turretMotor.setAllowableClosedLoopErr(7);
		turretMotor.configPeakOutputVoltage(+6.0f, -6.0f);
		turretMotor.enableForwardSoftLimit(false);
		turretMotor.enableReverseSoftLimit(false);
		
		turretMotor.setMotionMagicCruiseVelocity(2000);
		turretMotor.setMotionMagicAcceleration(2000);
		turretMotor.setProfile(1);
		turretMotor.setP(0.5);
		turretMotor.setI(0);
		turretMotor.setD(0.3);
		turretMotor.setF(0.3);
		turretMotor.setProfile(0);
		
		try
		{
			visionPipeline = new GripPipeline();
			visionListener = new VisionListener();
			visionCamera = CameraServer.getInstance().addAxisCamera(Constants.AXIS_IP);
			
			visionThread = new VisionThread(visionCamera, visionPipeline, visionListener);
			visionThread.start();
			
			isVisionAvailable = true;
		}catch(Exception e){
			e.printStackTrace();
			isVisionAvailable = false;
		}
		
	}
	
	/**Sets turret profile to zero*/
	public void zeroProfile() { turretMotor.setProfile(0); }
	
	/**
	 * Gets the required rpm using the shooter parabola
	 * @return The required rpm
	 */
	public double getRPM()
	{
		double rpm;
		
		double topRightY = visionListener.getTopRightPoint().y;
		
		if(Constants.IS_PRACTICE_BOT){
			//rpm = 298.36*distance + 3180.6;
			rpm = 0.0763 * topRightY * topRightY - 28.678 * topRightY + 7450;
		}else{
			//rpm = 61.49*distance*distance-561.5*distance+6419;
			//rpm = 0.1493*topRight.y*topRight.y-73.296*topRight.y+13931;
			//rpm = 0.159*topRight.y*topRight.y-78.558*topRight.y+14705;
			//rpm = 0.1722*topRight.y*topRight.y-86.499*topRight.y+15828+38; //quals
			//rpm = 0.1135*topRight.y*topRight.y-53.514*topRight.y+11276; //elims
			//rpm = 0.1352*topRight.y*topRight.y-63.866*topRight.y+12602; //practice match 1 - 25 ball auto
			//rpm = 0.1293*topRight.y*topRight.y-60.376*topRight.y+12088; //practice match 2 - middle gear + right balls - too long
			//rpm = 0.1234*topRight.y*topRight.y-56.886*topRight.y+11574; //match 1 - middle gear + right balls - too short
			//rpm = 0.1325*topRight.y*topRight.y-61.178*topRight.y+12078; //match 3 - too short
			//rpm = 0.1135*topRight.y*topRight.y-47.642*topRight.y+9921.4; //last with auto changing distance
			//rpm = 0.0975*topRight.y*topRight.y-38.543*topRight.y+8832.9; //21 balls auto
			//rpm = 0.0975*topRight.y*topRight.y-38.543*topRight.y+(8858 - 5); //district champs
			rpm = 0.0972 * topRightY * topRightY - 39.338 * topRightY + 8965.1; //worlds
		}
		
		if(rpm > 7000) rpm = 7000;
		return rpm;
	}
	
	/**Adds the amount to the shooter rpm offset*/
	public void adjustRPM(double amount) { shooterRPMOffset += amount; }
	
	/**
	 * Spins the shooter at the desired rpm
	 * @param rpm The desired rpm
	 */
	public void shoot(double rpm)
	{
		rpm += shooterRPMOffset;
		
		shooterMasterMotor.enable();
		shooterMasterMotor.changeControlMode(TalonControlMode.Speed);
		shooterMasterMotor.setF(0.5 * (1023 * 600) / (rpm * 4096));
		shooterMasterMotor.set(rpm);
		
		SmartDashboard.putNumber("TargetRPM", rpm);
	}
	
	/**Spins the shooter at the calculated rpm*/
	public void shoot() { shoot(getRPM()); }
	
	/**Stops shooter from spinning*/
	public void stopShoot() 
	{ 
		shooterMasterMotor.changeControlMode(TalonControlMode.PercentVbus);
		shooterMasterMotor.set(0); 
	}
	
	/**
	 * Moves the turret by the desired amount;
	 * @param amount The desired amount
	 */
	private void moveTurret(double amount)
	{
		boolean turn = true;
		
		boolean isForSwitched = turretMotor.getFaultForLim() == 1;
		boolean isRevSwitched = turretMotor.getFaultRevLim() == 1;
		
		if(isForSwitched){
			turn = (amount < 0);
			autoTurretDirection = -1;
			Constants.FOR_LIMIT_POSITION = turretMotor.getEncPosition();
		}else if(isRevSwitched){
			turn = (amount > 0);
			autoTurretDirection = 1;
			Constants.REV_LIMIT_POSITION = turretMotor.getEncPosition();
		}
		
		if(turn)
		{
			turretPosition = turretMotor.getPosition() + amount;
			zeroProfile();
			turretMotor.set(turretPosition);
		}
	}
	
	/**
	 * Adjusts the turret by a small amount using the desired amount
	 * @param amount The desired amount
	 */
	public void adjustTurret(double amount)
	{
		if(turretTarget == 0)
			turretTarget = getTurretPosition();
		
		turretTarget += (amount / 4096);
	}
	
	/**
	 * Turns the turret by the desired amount
	 * @param ticks The desired amount in ticks
	 * @param override True if vision centering should be override
	 */
	public void turnTurret(double ticks, boolean overrideVision)
	{
		if(turretTimerRunning == 0)
		{
			double turnTicks;
			
			if(visionListener.isBoilerVisible() && !overrideVision)
			{
				double centerOffset = (Constants.IMAGE_WIDTH / 2) - visionListener.getCenterX();
				double degreeOffset = Constants.AXIS_FOV * (centerOffset / Constants.IMAGE_WIDTH);
				turnTicks = ((degreeOffset / Constants.TURRET_ROTATION_DEG) * Constants.TURRET_ROTATION_TICKS) / 4096;
				
				SmartDashboard.putNumber("centerOffset", centerOffset);
				SmartDashboard.putNumber("degreeOffset", degreeOffset);
				
				turnTicks /= 5;
			}
			else
				turnTicks = ticks / 2;
			
			if(Math.abs(turnTicks) > 0.5) 
				turnTicks = Math.signum(turnTicks) * 0.5;
			
			turretTarget = 0;
			moveTurret(turnTicks);
		}
	}
	
	/**
	 * Turns the to the desired degree
	 * @param degree The desired degree
	 * @return 1 = turn is finished; 0 = still turning
	 */
	public int turnTurret(double degree)
	{
		degreeTarget = (Constants.FOR_LIMIT_POSITION + (degree / Constants.TURRET_ROTATION_DEG) * Constants.TURRET_ROTATION_TICKS) / 4096;
		
		if(!isDegreeTurning)
		{
			turretMotor.changeControlMode(TalonControlMode.MotionMagic);
			
			turretMotor.setF(0.5);
			
			turretMotor.setMotionMagicCruiseVelocity(1000);
			turretMotor.setMotionMagicAcceleration(500);
			
			zeroProfile();
			
			turretTarget = 0;
			turretMotor.set(degreeTarget);
		}
		
		isDegreeTurning = (turretMotor.getClosedLoopError() < 50);
		return (isDegreeTurning ? 1 : 0);
	}
	
	/**@return The position of the turret*/
	public double getTurretPosition() { return turretMotor.getPosition(); }
	
	/**Moves the turret to the desired position using vision. */
	public void alignTurret()
	{
		shooterRPMOffset = 0;
		turretTarget = getTurretPosition();
		
		if(!isVisionCentering)
		{
			visionCenteringTimer = new Timer();
			visionCenteringTimer.schedule(new VisionCenteringTask(), 0, 20L);
			isVisionCentering = true;
		}
	}
	
	private class VisionCenteringTask extends TimerTask
	{
		private int turretCenteringCounter = 0;
		
		private int size;
		private double samples[];
		
		private int index = 0;
		
		private double min = Double.MAX_VALUE;
		private double max = -1 * Double.MAX_VALUE;
		
		private boolean isFound = false;
		
		public VisionCenteringTask()
		{
			turretCenteringCounter = 0;
			
			size = 10;
			samples = new double[size];
			for(int i = 0; i < size; i++) samples[i] = 0;
			
			isFound = false;
		}
		
		@Override
		public void run() 
		{
			if(turretTimerRunning == 0 || turretTimerRunning == 1)
			{
				turretTimerRunning = 1;
				double turnTicks = 0;
				
				SmartDashboard.putBoolean("turretCentered", false);
				
				if(visionListener.getFrameCount() != lastFrame)
				{
					lastFrame = visionListener.getFrameCount();
					
					if(visionListener.isBoilerVisible())
					{
						if(turretCenteringCounter > 20 && !isFound)
						{
							addSample(turretMotor.getPosition());
							
							if(Math.abs(turretMotor.getSpeed()) < 0.1)
							{
								turretCenteringCounter++;
								
								double center = cameraOffset;
								double centerOffset = center - visionListener.getCenterX();
								double degreeOffset = Constants.AXIS_FOV * (centerOffset / Constants.IMAGE_WIDTH);
								
								turnTicks = 0.7 * (((degreeOffset) / Constants.TURRET_ROTATION_DEG) * Constants.TURRET_ROTATION_TICKS) / 4096;
								double turretSet = turretMotor.getPosition() + turnTicks;
								
								if(Math.abs(centerOffset) > 7)
								{
									zeroProfile();
									turretTarget = 0;
									
									turretMotor.set(turretSet);
								}
								else
									isFound = true;
							}
							
						}else{
							System.out.println(System.nanoTime() + " centeringDone");
			            	System.out.println(System.nanoTime() + " " + turretCenteringCounter);
			            	
			    			turretTimerRunning = 0;
			    			isVisionCentering = false;
			    			turretTarget = getTurretPosition();
			    			
			    			visionCenteringTimer.cancel();
			    			visionCenteringTimer.purge();
						}
					
					}else if(Math.abs(turretMotor.getSpeed()) < 5){
						turnTicks = autoTurretDirection;
						moveTurret(turnTicks);
					}
				}
				
				SmartDashboard.putNumber("CenterX", visionListener.getCenterX());
		    	SmartDashboard.putNumber("REVLimit_num", Constants.REV_LIMIT_POSITION);
		    	SmartDashboard.putNumber("FWDLimit_num", Constants.FOR_LIMIT_POSITION);

			}
			else 
				System.out.println("Center Turret - Turret Timer Running");
			
		}
		
		/**
		 * Add a position to the sample positions
		 * @param position The sample position
		 */
		private void addSample(double position)
		{
			samples[index] = position;
			
			if(++index == size) index = 0;
			
			min = Double.MAX_VALUE;
			max = -1 * Double.MAX_VALUE;
			
			for(int i = 0; i < size; i++)
			{
				if(samples[i] > max) max = samples[i]; 
		    	if(samples[i] < min) min = samples[i]; 
			}
		}
	}
	
	/**
	 * Finds the limit switch if not found already
	 * @return True if limit switch has been found
	 */
	public boolean findRevLimit()
	{
		if(!isFindingLimitSwitch && !isLimitSwitchFound)
		{
			findLimitSwitchTimer = new Timer();
			findLimitSwitchTimer.schedule(new FindLimitSwitchTask(), 0, 5L);
			isFindingLimitSwitch = true;
		}
		
		return isLimitSwitchFound;
	}
	
	private class FindLimitSwitchTask extends TimerTask
	{
		double turnTicks = 0;

		@Override
		public void run() 
		{
			if(turretTimerRunning == 0 || turretTimerRunning == 2)
			{
				turretTimerRunning = 2;
				
				if(!isLimitSwitchFound)
					turnTicks = 0.5;
				
				moveTurret(turnTicks);
				
				if(Constants.FOR_LIMIT_POSITION < Integer.MAX_VALUE){
		    		Constants.FOR_LIMIT_POSITION = turretMotor.getEncPosition();
		    		turretTimerRunning = 0;
		    		isLimitSwitchFound = true;
		    		findLimitSwitchTimer.cancel();
		    		findLimitSwitchTimer.purge();
		    	}
			}
		}
	}
	
	public void stopTimer()
	{
		if(visionCenteringTimer != null)
			visionCenteringTimer.cancel();
		
		if(findLimitSwitchTimer != null)
			findLimitSwitchTimer.cancel();
		
		turretTimerRunning = 0;
		isVisionCentering = false;
	}
	
	/**
	 * Spins the ball pump to the desired rpm
	 * @param rpm The desired rpm
	 */
	public void setBallPump(double rpm) { ballPumpMotor.set(rpm); }
	
	/**Stops the ball pump from spinning*/
	public void stopBallPump() { ballPumpMotor.set(0); }
	
	/**
	 * Spins the indexer at the desired rpm
	 * @param rpm The desired rpm
	 */
	public void setIndexer(double rpm)
	{
		indexerMotor.changeControlMode(TalonControlMode.Speed);
		indexerMotor.set(rpm);
	}
	
	/**Stops the indexer from spinning*/
	public void stopIndexer() { indexerMotor.set(0); }
	
	public void setCameraOffset(double cameraOffset) { this.cameraOffset = cameraOffset; }
	
	public boolean isVisionCentering() { return isVisionCentering; }
	
	public double getImageTop() { return visionListener.getTopRightPoint().y; }
	
	/**
	 * Checks if camera is connected
	 * @return True if vision is working
	 */
	public boolean isVisionAvailable() { return isVisionAvailable; }
	
	/**Display limit switches on dashboard*/
	public void displayLimitSwitches()
	{
		SmartDashboard.putBoolean("FWD Limit", turretMotor.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("REV Limit", turretMotor.isRevLimitSwitchClosed());
	}
	
	/**Display shooter values on dashboard*/
	public void displayShooter()
	{
		SmartDashboard.putNumber("TurretEncPos", turretMotor.getEncPosition());
		SmartDashboard.putNumber("TurretPos", turretMotor.getPosition());
		SmartDashboard.putNumber("TurretPositionSet", turretPosition);
    	SmartDashboard.putNumber("RPM", shooterMasterMotor.getSpeed());
	}
	
}
