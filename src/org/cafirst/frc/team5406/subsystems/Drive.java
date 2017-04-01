package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.robot.Gearbox;
import org.cafirst.frc.team5406.robot.Motor;
import org.cafirst.frc.team5406.auto.MPDataLeftHopper;
import org.cafirst.frc.team5406.auto.MotionProfile;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import java.util.Timer;
import java.util.TimerTask;



public class Drive extends Subsystems{
	private CANTalon[] leftDriveMotors;
	private CANTalon[] rightDriveMotors;
    private DoubleSolenoid shiftSolenoid;
    private PowerDistributionPanel pdp;
    public boolean precisionDriveX = false;
    public boolean precisionDriveY = false;
    private boolean highGear = false;
    
	/** some example logic on how one can manage an MP */
	private MotionProfile _left;
	private MotionProfile _right;
	private Timer PIDTimer = new Timer();
	private PIDLoop anglePID;


	


	
	private MPDataLeftHopper mpData = new MPDataLeftHopper();



	RobotDrive drive;
	public Drive(){
				
		leftDriveMotors = InitializeMotors(Constants.LEFT_DRIVE);
		rightDriveMotors = InitializeMotors(Constants.RIGHT_DRIVE);
		leftDriveMotors[0].setVoltageRampRate(100);
		rightDriveMotors[0].setVoltageRampRate(100);
		shiftSolenoid = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
		drive = new RobotDrive(leftDriveMotors[0], rightDriveMotors[0]);
		pdp = new PowerDistributionPanel();
		
		/** some example logic on how one can manage an MP */
		_left = new MotionProfile(leftDriveMotors[0], mpData.PointsLeft, mpData.kNumPoints);
		_right = new MotionProfile(rightDriveMotors[0], mpData.PointsRight, mpData.kNumPoints);


		
	}
	
	public void ArcadeDrive(double x, double y){
		leftDriveMotors[0].enable();
		rightDriveMotors[0].enable();
		if (precisionDriveX){
			x *=0.5;
		}		
		if (precisionDriveY){
			y *=0.5;
		}
		drive.arcadeDrive(x, y);
	}
	
	public void DisplayCurrent(){
		DisplayCurrent(leftDriveMotors);
		DisplayCurrent(rightDriveMotors);
	}
	
	public void shiftHigh(){
		shiftSolenoid.set(Constants.SHIFT_HIGH);
		highGear = true;
	}
	public void shiftLow(){
		shiftSolenoid.set(Constants.SHIFT_LOW);
		highGear = false;
	}
	
	
	public void AutoInit(){
		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_left.control();
		_right.control();
		
		leftDriveMotors[0].changeControlMode(TalonControlMode.MotionProfile);
			
			CANTalon.SetValueMotionProfile setOutputLeft = _left.getSetValue();
					
			leftDriveMotors[0].set(setOutputLeft.value);

			rightDriveMotors[0].changeControlMode(TalonControlMode.MotionProfile);
			
			CANTalon.SetValueMotionProfile setOutputRight = _right.getSetValue();
					
			rightDriveMotors[0].set(setOutputRight.value);	
			_left.startMotionProfile();
			_right.startMotionProfile();
	}
	
	
	public void driveAtAngleInit(double _speed, double _angle, boolean _correct){
		PIDTimer = new Timer();
		anglePID = new PIDLoop(_speed,_angle,_correct);
	    PIDTimer.schedule(anglePID, 0L, 10L); //time in milliseconds

	}
	public void driveAtAngleUpdate(double _speed, double _angle, boolean _correct){
		anglePID.updateValues(_speed, _angle, _correct);
	}
	
	public void driveAtAngleEnd(){
		PIDTimer.cancel();
		leftDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		rightDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
	}
	

	
	public double[] getPosition(){
		double[] currentPosition = {leftDriveMotors[0].getPosition(), rightDriveMotors[0].getPosition()};
		return currentPosition;
	}
	
	class PIDLoop extends TimerTask {
	private double angle;
	private boolean correct;
	private double speed;
	private double accumI = 0.0;
	public double lastAngle = 0;
	private double previousError = 0.0;
	
	public PIDLoop(double _speed, double _angle, boolean _correct){
		speed = _speed;
		angle = _angle;
		correct = _correct;
	}
	
	public void updateValues(double _speed, double _angle, boolean _correct){
		speed = _speed;
		angle = _angle;
		correct = _correct;
	}
	
	
	public void run(){
		double leftSpeed = 0;
		double rightSpeed = 0;

		System.out.println("Angle: " + Constants.navX.getYaw());
		double currentAngle = Constants.navX.getYaw();
		//change in encoder value
		double dSpeed = 0;
		double speedChangeMultiplier = 0;
		if(correct){
		speedChangeMultiplier = calcSpeed(angle - currentAngle);
	    	System.out.println("speedChangeMultiplier" + speedChangeMultiplier);
	    	dSpeed = speed*speedChangeMultiplier;
		}
		SmartDashboard.putNumber("speedChangeMultiplier", speedChangeMultiplier);
		SmartDashboard.putNumber("Heading-DriveStraight", Constants.navX.getYaw());
		leftSpeed = -1*speed-dSpeed;
		rightSpeed = speed-dSpeed;
		
		SmartDashboard.putNumber("Left Speed", leftSpeed);
		SmartDashboard.putNumber("Right Speed", rightSpeed);
		
		leftDriveMotors[0].changeControlMode(TalonControlMode.Speed);
		leftDriveMotors[0].set( leftSpeed);
		rightDriveMotors[0].changeControlMode(TalonControlMode.Speed);
		rightDriveMotors[0].set( rightSpeed);

	}
	
	
    public double calcSpeed(double currentError){
		
 		double valP = Constants.GYRO_PID_P * currentError;
 		double valI = accumI;
 		double valD = Constants.GYRO_PID_D * (previousError - currentError);
 		if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
 		accumI += Constants.GYRO_PID_I;
 		
 		//If we overshoot, reset the I
 		if(Math.signum(previousError) != Math.signum(currentError)){ 
 			accumI = 0; 
 			valI = 0;
 		}
 		
 		double speed = valP + (valI * (currentError > 0 ? 1.0 : -1.0)) - valD;

 		previousError = currentError;
 		
 		return speed;
 	}
	}
 
	
	public void resetPosition(){
		leftDriveMotors[0].setPosition(0);
		rightDriveMotors[0].setPosition(0);
	}
	
	public void enableBrake(boolean enable){
		leftDriveMotors[0].enableBrakeMode(enable);
		rightDriveMotors[0].enableBrakeMode(enable);
	}
	
	
	
	public int AutoLoop(){
		_left.control();
		_right.control();
		
		
		CANTalon.SetValueMotionProfile setOutputLeft = _left.getSetValue();
				
		leftDriveMotors[0].set(setOutputLeft.value);
		if(setOutputLeft.value==2){
			leftDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
			leftDriveMotors[0].set( 0 );
		}
		
		CANTalon.SetValueMotionProfile setOutputRight = _right.getSetValue();
				
		rightDriveMotors[0].set(setOutputRight.value);	
		if(setOutputRight.value==2){
			rightDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
			rightDriveMotors[0].set( 0 );
		}
		return setOutputRight.value;
	
	}
	
	public void disableAuto(){
		/* it's generally a good idea to put motor controllers back
		 * into a known state when robot is disabled.  That way when you
		 * enable the robot doesn't just continue doing what it was doing before.
		 * BUT if that's what the application/testing requires than modify this accordingly */
		leftDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		leftDriveMotors[0].set( 0 );
		/* clear our buffer and put everything into a known state */
		_left.reset();
		rightDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		rightDriveMotors[0].set( 0 );
		/* clear our buffer and put everything into a known state */
		_right.reset();
	}
	

}
