package org.cafirst.frc.team5406.subsystem;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.util.Constants;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;

public class Drive extends Subsystem {

	private CANTalon leftMasterDrive;
	private CANTalon[] leftSlaveDrive = new CANTalon[2];
	
	private CANTalon rightMasterDrive;
	private CANTalon[] rightSlaveDrive = new CANTalon[2];
	
	private DoubleSolenoid shiftSolenoid;
	
	private RobotDrive drive;
	
	private boolean precisionDriveX = false;
	private boolean precisionDriveY = false;
	
	private Timer angleDriveTimer;
	private AngleDriveTask angleDriveTask;
	
	private Timer curveDriveTimer;
	private CurveDriveTask curveDriveTask;
	private boolean curveStatus = false;
	
	public Drive() {
		super(Constants.drive);
		
		leftMasterDrive = this.initializeMotors(Constants.LEFT_DRIVE_MASTER_ID, true, 0);
		rightMasterDrive = this.initializeMotors(Constants.RIGHT_DRIVE_MASTER_ID, true, 0);
		leftMasterDrive.setAllowableClosedLoopErr(50);
		rightMasterDrive.setAllowableClosedLoopErr(50);
		this.setVoltageRampRate(100);
		
		for(int i = 0; i < leftSlaveDrive.length; i++)
		{
			leftSlaveDrive[i] = this.initializeMotors(Constants.LEFT_DRIVE_FOLLOWER_IDS[i], false, Constants.LEFT_DRIVE_MASTER_ID);
			rightSlaveDrive[i] = this.initializeMotors(Constants.RIGHT_DRIVE_FOLLOWER_IDS[i], false, Constants.RIGHT_DRIVE_MASTER_ID);
		}
		
		shiftSolenoid = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
		
		drive = new RobotDrive(leftMasterDrive, rightMasterDrive);
	}
	
	/**
	 * Sets the voltage ramp rate for the 2 Master Drive Motors.
	 * @param rampRate The desired ramp rate.
	 */
	public void setVoltageRampRate(double rampRate)
	{
		leftMasterDrive.setVoltageRampRate(rampRate);
		rightMasterDrive.setVoltageRampRate(rampRate);
	}
	
	/**
	 * Drives the robot using an x value and y value for power.
	 * @param x The desired x value. 1 is 100%.
	 * @param y The desired y value. 1 is 100%.
	 */
	public void arcadeDrive(double x, double y)
	{
		leftMasterDrive.enable();
		rightMasterDrive.enable();
		
		if(precisionDriveX)
			x *= 0.5;
		if(precisionDriveY)
			y *= 0.5;
		
		drive.arcadeDrive(x, y);
	}
	
	/**Shifts the robot into high gear*/
	public void shiftHigh() { shiftSolenoid.set(Constants.SHIFT_HIGH); }
	/**Shifts the robot into low gear*/
	public void shiftLow() { shiftSolenoid.set(Constants.SHIFT_LOW); }
	
	/**Resets all the position values on the Talons.*/
	public void resetPosition()
	{
		leftMasterDrive.setPosition(0);
		rightMasterDrive.setPosition(0);
	}
	
	/**
	 * Toggles the brake mode on the Talons
	 * @param enable True if brakes should be enabled
	 */
	public void toggleBrakeMode(boolean enable)
	{
		leftMasterDrive.enableBrakeMode(enable);
		rightMasterDrive.enableBrakeMode(enable);
	}
	
	public void setPrecisionDriveX(boolean precisionDriveX) { this.precisionDriveX = precisionDriveX; }
	public void setPrecisionDriveY(boolean precisionDriveY) { this.precisionDriveY = precisionDriveY; }
	
	/**Gets the current position values on the Talon. 0 is right, 1 is left.*/
	public double[] getPosition()
	{
		double currPosition[] = new double[2];
		if(Constants.IS_PRACTICE_BOT)
		{
			currPosition[0] = leftMasterDrive.getPosition();
			currPosition[1] = rightMasterDrive.getPosition();
		}else{
			currPosition[0] = rightMasterDrive.getPosition();
			currPosition[1] = leftMasterDrive.getPosition();
		}
		
		return currPosition;
	}
	
	/**Gets the current speed values on the Talon. 0 is right, 1 is left*/
	public double[] getSpeed()
	{
		double[] currSpeed = new double[2];
		if(Constants.IS_PRACTICE_BOT)
		{
			currSpeed[0] = leftMasterDrive.getSpeed();
			currSpeed[1] = rightMasterDrive.getSpeed();
		}else{
			currSpeed[0] = rightMasterDrive.getSpeed();
			currSpeed[1] = leftMasterDrive.getSpeed();
		}
		
		return currSpeed;
	}
	
	/**
	 * Starts the Drive at Angle Timer.
	 * @param speed The desired speed in RPM.
	 * @param angle The desired driving angle. -180 to 180.
	 * @param correct True if angle correction is desired.
	 */
	public void driveAtAngleStart(double speed, double angle, boolean correct)
	{
		angleDriveTimer = new Timer();
		angleDriveTask = new AngleDriveTask(speed, angle, correct, 0);
		angleDriveTimer.schedule(angleDriveTask, 0, 10L);
	}
	
	/**
	 * Updates the values for the DriveAtAngleTimer.
	 * @param speed The desired speed in RPM.
	 * @param angle The desired driving angle. -180 to 180.
	 * @param correct True if angle correction is desired.
	 */
	public void driveAtAngleUpdate(double speed, double angle, boolean correct)
	{
		angleDriveTask.updateValues(speed, angle, correct, 0);
	}
	
	/**Ends the current DriveAtAngle Timer*/
	public void driveAtAngleEnd()
	{
		if(angleDriveTimer != null)
			angleDriveTimer.cancel();
		
		leftMasterDrive.changeControlMode(TalonControlMode.PercentVbus);
		rightMasterDrive.changeControlMode(TalonControlMode.PercentVbus);
		leftMasterDrive.set(0);
		rightMasterDrive.set(0);
	}
	
	private class AngleDriveTask extends TimerTask
	{
		private double angle;
		private boolean correct;
		private double speed;
		private double offset;
		private double accumI = 0.0;
		private double previousError = 0.0;
		
		/**
		 * Constructor for AngleDriveTask
		 * @param speed The desired speed in RPM.
		 * @param angle The desired driving angle. -180 to 180.
		 * @param correct True if angle correction is desired.
		 * @param offset The desired offset. Used in curve timer
		 */
		public AngleDriveTask(double speed, double angle, boolean correct, double offset)
		{
			this.speed = speed;
			this.angle = angle;
			this.correct = correct;
			this.offset = offset;
		}
		
		/**
		 * Updates the values for the DriveAtAngleTimer.
		 * @param speed The desired speed in RPM.
		 * @param angle The desired driving angle. -180 to 180.
		 * @param correct True if angle correction is desired.
		 * @param offset The desired offset. Used in curve timer
		 */
		public void updateValues(double speed, double angle, boolean correct, double offset)
		{
			this.speed = speed;
			this.angle = angle;
			this.correct = correct;
			this.offset = offset;
		}

		@Override
		public void run() 
		{
			double leftSpeed = 0;
			double rightSpeed = 0;
			
			double currentAngle = Constants.navX.getYaw();
			
			double dSpeed = 0;
			double speedChangeMultiplier = 0;
			
			if(correct)
			{
				System.out.println("Angle: " + angle + ", currentAngle: " + currentAngle);
				
				speedChangeMultiplier = calcSpeed(angle - currentAngle);
				dSpeed = speed * (offset + speedChangeMultiplier);
			}
			
			leftSpeed = -1 * speed - Math.signum(speed) * dSpeed;
			rightSpeed = speed - Math.signum(speed) * dSpeed;
			
			leftMasterDrive.changeControlMode(TalonControlMode.Speed);
			leftMasterDrive.set(leftSpeed);
			rightMasterDrive.changeControlMode(TalonControlMode.Speed);
			rightMasterDrive.set(rightSpeed);
		}
		
		/**
		 * Calculates the desired speed
		 * @param currentError The current error
		 * @return The calculated speed
		 */
		public double calcSpeed(double currentError)
		{
			double speed;
			
			double valP = Constants.GYRO_PID_P * currentError;
			double valI = accumI;
			double valD = Constants.GYRO_PID_D * (previousError - currentError);
			
			//Limit so that D isn't the driving number
			if(Math.abs(valD) > Math.abs(valP)) valD = valP; 
			
			accumI += Constants.GYRO_PID_I;
			
			//If we overshoot, reset the I
			if(Math.signum(previousError) != Math.signum(currentError))
			{
				accumI = 0;
				valI = 0;
			}
			
			speed = valP + (valI * (currentError > 0 ? 1.0 : -1.0)) - valD;
			
			previousError = currentError;
			
			return speed;
		}
		
	}
	
	/**
	 * Starts the Drive Along Curve Timer
	 * @param speed The desired speed RPM
	 * @param radius The radius of the desired circle
	 * @param targetAngle The angle the robot ends at in the circle
	 * @param coastDistance The coast distance
	 */
	public void driveAtCurveStart(double speed, double radius, double targetAngle, double coastDistance)
	{
		curveDriveTimer = new Timer();
		curveDriveTask = new CurveDriveTask(speed, radius, targetAngle, coastDistance, curveDriveTimer);
		curveDriveTimer.schedule(curveDriveTask, 0, 5L);
	}
	
	/**Ends the CurveDriveTimer*/
	public void driveAtCurveEnd()
	{
		if(curveDriveTimer != null)
			curveDriveTimer.cancel();
	}
	
	/**@return True if robot has finished the desired curve*/
	public boolean isCurveCompleted() { return curveStatus; }
	
	private class CurveDriveTask extends TimerTask
	{

		double speed;
		double radius;
		double targetAngle;
		double coastDistance;
		
		double ffSpeedOffset;
		double startAngle;
		double dAngle;
		double start;
		double pathLength;
		double startCurvePercent;
		
		Timer curveTimer;
		
		/**
		 * Constructor for CurveDriveTask. AngleDriveTimer must be running.
		 * @param speed The desired speed RPM
	 	 * @param radius The radius of the desired circle
	 	 * @param targetAngle The angle the robot ends at in the circle
	 	 * @param coastDistance The coast distance
		 * @param curveTimer The CurveDriveTimer
		 */
		public CurveDriveTask(double speed, double radius, double targetAngle, double coastDistance, Timer curveTimer)
		{
			this.speed = speed;
			this.radius = radius;
			this.targetAngle = targetAngle;
			this.coastDistance = coastDistance;
			this.curveTimer = curveTimer;
			
			curveStatus = false;
			
			startAngle = Constants.navX.getYaw();
			dAngle = targetAngle - startAngle;
			ffSpeedOffset = calcFFSpeedOffset(this.radius);
			
			double[] robotPosition = getPosition();
			start = (robotPosition[0] + robotPosition[1]) / 2;
			pathLength = (Math.PI * radius * (Math.abs(dAngle))) / 180;
			startCurvePercent = this.coastDistance / (this.coastDistance + pathLength);
		}
		
		@Override
		public void run() 
		{
			double[] robotPosition = getPosition();
			double robotPositionAvg = (robotPosition[0] + robotPosition[1]) / 2;
			
			double _speed = speed;
			
			double percentDone = Math.abs(robotPositionAvg - start) / (Math.abs(pathLength + coastDistance) / (Constants.WHEEL_DIAM * Math.PI));
			double percentCurve = (Math.abs(robotPositionAvg - start) - (coastDistance / (Constants.WHEEL_DIAM * Math.PI))) / (Math.abs(pathLength) / (Constants.WHEEL_DIAM * Math.PI));
			double setAngle;
			
			if(percentDone < startCurvePercent)
				setAngle = startAngle + dAngle / 20;
			else if (percentDone >= 1)
			{
				setAngle = targetAngle;
				ffSpeedOffset = 0;
				curveStatus = true;
				curveTimer.cancel();
				curveTimer.purge();
			}
			else
				setAngle = startAngle + dAngle * percentCurve;
			
			angleDriveTask.updateValues(_speed, setAngle, true, ffSpeedOffset);
		}
		
		/**@return The calculated speed offset*/
		public double calcFFSpeedOffset(double radius)
		{
			double offsetAdjustment = 0.9;
			return (Math.signum(dAngle) < 1 ? -1 : 1) * Constants.WHEEL_TRACK / (2 * radius) * offsetAdjustment;
		}
	}
}
