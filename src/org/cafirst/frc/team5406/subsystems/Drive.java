package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.robot.Gearbox;
import org.cafirst.frc.team5406.robot.Motor;

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
	private Timer PIDTimer = new Timer();
	private PIDLoop anglePID;
	private Timer curveTimer = new Timer();
	private driveCurveMonitor dcMon;
	private boolean curveStatus = false;

	




	RobotDrive drive;
	public Drive(){
				
		leftDriveMotors = InitializeMotors(Constants.LEFT_DRIVE);
		rightDriveMotors = InitializeMotors(Constants.RIGHT_DRIVE);
		leftDriveMotors[0].setAllowableClosedLoopErr(50);
		rightDriveMotors[0].setAllowableClosedLoopErr(50);

		setVolageRampRate(100);
		shiftSolenoid = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
		drive = new RobotDrive(leftDriveMotors[0], rightDriveMotors[0]);
		pdp = new PowerDistributionPanel();
		
		/** some example logic on how one can manage an MP */


		
	}
	
	public void setVolageRampRate(double rate){
		leftDriveMotors[0].setVoltageRampRate(rate);
		rightDriveMotors[0].setVoltageRampRate(rate);

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
	
	

	
	
	public void driveAtAngleInit(double _speed, double _angle, boolean _correct){
		PIDTimer = new Timer();
		anglePID = new PIDLoop(_speed,_angle,_correct, 0);
	    PIDTimer.schedule(anglePID, 0L, 10L); //time in milliseconds

	}
	public void driveAtAngleUpdate(double _speed, double _angle, boolean _correct){
		anglePID.updateValues(_speed, _angle, _correct, 0);
	}
	
	public void driveAtAngleInit(double _speed, double _angle, boolean _correct, double _offset){
		PIDTimer = new Timer();
		anglePID = new PIDLoop(_speed,_angle,_correct, _offset);
		leftDriveMotors[0].changeControlMode(TalonControlMode.Speed);
		rightDriveMotors[0].changeControlMode(TalonControlMode.Speed);

	    PIDTimer.schedule(anglePID, 0L, 10L); //time in milliseconds

	}
	public void driveAtAngleUpdate(double _speed, double _angle, boolean _correct, double _offset){
		anglePID.updateValues(_speed, _angle, _correct, _offset);
	}
	
	public void driveAtAngleEnd(){
		if(PIDTimer != null){
			PIDTimer.cancel();
		}
		leftDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		rightDriveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		leftDriveMotors[0].set(0);
		rightDriveMotors[0].set(0);
		
	}
	

	public void driveAlongCurveInit(double _speed, double _radius, double _targetAngle, double _coastDistance){
		System.out.println("CurveInit");
		curveTimer = new Timer();
		dcMon = new driveCurveMonitor(_speed,_radius,_targetAngle, _coastDistance);
	    curveTimer.schedule(dcMon, 0L, 2L); //time in milliseconds

	}
	
	
	public boolean driveAlongCurveCompleted(){
		return curveStatus;
	}
	

	public void driveAlongCurveEnd(){
		if(curveTimer != null){
			curveTimer.cancel();
		}
	}
	
	
	class driveCurveMonitor extends TimerTask {
		double radius;
		double speed;
		double targetAngle;
		double ffSpeedOffset;
		double startAngle;
		double dAngle;
		double start;
		double pathLength;
		double end;
		double coastDistance;
		double startCurve;
		double startCurvePercent;
	
	public double calcFFSpeedOffset(double radius){
		double offsetAdjustment = 0.9;
		System.out.println("dAngle: " + dAngle + " WheelTrack: " + Constants.WHEEL_TRACK + ", radius: "+radius);
		
		return (Math.signum(dAngle)<1?-1:1)*Constants.WHEEL_TRACK/(2*radius)*offsetAdjustment;
		//return 0.31;
	}
	
	public driveCurveMonitor(double _speed, double _radius, double _targetAngle, double _coastDistance){
		curveStatus = false;
		speed = _speed;
		radius = _radius;
		targetAngle = _targetAngle;
		coastDistance = _coastDistance;
		
		System.out.println("ffOffsetCalc: " + ffSpeedOffset);
		startAngle = Constants.navX.getYaw();
		dAngle = _targetAngle-startAngle;
		ffSpeedOffset = calcFFSpeedOffset(radius);
		double[] robotPosition = getPosition();
		/*start = (robotPosition[0]+robotPosition[1])/2;
		startCurve = start + Math.signum(speed)*coastDistance;
		pathLength = Math.signum(speed)*(Math.PI*_radius*(Math.abs(dAngle)))/180;
		end = startCurve+(pathLength/(Constants.WHEEL_DIAM*Math.PI));*/
		start = (robotPosition[0]+robotPosition[1])/2;
		pathLength = (Math.PI*_radius*(Math.abs(dAngle)))/180;
		startCurvePercent = coastDistance/(coastDistance+pathLength);
		
		
	}

	
	
	public void run(){
		double[] robotPosition = getPosition();
		double robotPositionAvg = ((robotPosition[0]+robotPosition[1])/2);
		double _speed = speed;
		//double setAngle = startAngle + dAngle*(robotPositionAvg-startCurve)/(pathLength/(Constants.WHEEL_DIAM*Math.PI)); //the angle we should be at this point along the path length
		double percentDone = Math.abs(robotPositionAvg-start)/(Math.abs(pathLength+coastDistance)/(Constants.WHEEL_DIAM*Math.PI));
		double percentCurve = (Math.abs(robotPositionAvg-start)-(coastDistance/(Constants.WHEEL_DIAM*Math.PI)))/(Math.abs(pathLength)/(Constants.WHEEL_DIAM*Math.PI));
		double setAngle;
		if(percentDone < startCurvePercent){
			setAngle = startAngle + dAngle/20;
		}else if (percentDone>=1){
			setAngle =targetAngle;
			//speed = 0;
			ffSpeedOffset = 0;
			curveStatus = true;
			curveTimer.cancel();
			curveTimer.purge();
		}else{
			setAngle = startAngle + dAngle*percentCurve; //the angle we should be at at this point along the path length

		}
		setAngle = startAngle + dAngle*percentCurve; //the angle we should be at at this point along the path length

		System.out.println(percentDone + "%   lPos: " + robotPosition[0] + ", rPos: " + robotPosition[1] + ", Current Angle: " + Constants.navX.getYaw() + "Set Angle: " + setAngle + ", start: " + start +", %Coast: " + startCurvePercent + ", %Curve: " + percentCurve);
		System.out.println("pathLength: " + pathLength +", dAngle: " + dAngle + ", targetAngle: " + targetAngle + ", ffSpeedOffset: " + ffSpeedOffset + ", robotPositionAvg: " + robotPositionAvg);
		anglePID.updateValues(_speed, setAngle, true, ffSpeedOffset); //calls driveAtAngle to match our new target
	}
	
	}
	
	public double[] getPosition(){
		double[] currentPosition = new double[2];
		if (Constants.IS_PRACTICE_BOT){
			currentPosition[0] = leftDriveMotors[0].getPosition();
			currentPosition[1] = rightDriveMotors[0].getPosition();
		}else{
			currentPosition[0] = rightDriveMotors[0].getPosition();
			currentPosition[1] = leftDriveMotors[0].getPosition();
		}
		SmartDashboard.putNumber("leftPosition", currentPosition[0]);
		SmartDashboard.putNumber("rightPosition", currentPosition[1]);
		return currentPosition;
	}
	
	public double[] getSpeed(){
		double[] currentSpeed = new double[2];
		if (Constants.IS_PRACTICE_BOT){
			currentSpeed[0] = leftDriveMotors[0].getSpeed();
			currentSpeed[1] = rightDriveMotors[0].getSpeed();
		}else{
			currentSpeed[0] = rightDriveMotors[0].getSpeed();
			currentSpeed[1] = leftDriveMotors[0].getSpeed();
		}
		return currentSpeed;
	}
	
	class PIDLoop extends TimerTask {
	private double angle;
	private boolean correct;
	private double speed;
	private double accumI = 0.0;
	public double lastAngle = 0;
	private double previousError = 0.0;
	private double offset = 0;
	
	public PIDLoop(double _speed, double _angle, boolean _correct, double _offset){
		speed = _speed;
		angle = _angle;
		correct = _correct;
		offset = _offset;
	}
	
	public void updateValues(double _speed, double _angle, boolean _correct, double _offset){
		speed = _speed;
		angle = _angle;
		correct = _correct;
		offset = _offset;
	}
	
	
	public void run(){
		double leftSpeed = 0;
		double rightSpeed = 0;

		//System.out.println("Angle: " + Constants.navX.getYaw());
		double currentAngle = Constants.navX.getYaw();
		//change in encoder value
		double dSpeed = 0;
		double speedChangeMultiplier = 0;
		if(correct){
			System.out.println("Angle: " + angle + ", currentAngle: " + currentAngle);
			SmartDashboard.putNumber("DriveAngleRes", angle - currentAngle);


		speedChangeMultiplier = calcSpeed(angle - currentAngle);
		System.out.println("SpeedChangeMultiplier: " + speedChangeMultiplier);
		System.out.println("Left Speed: " + 	leftDriveMotors[0].getSpeed() + ", TargetSpeed: " + speed);
		SmartDashboard.putNumber("Left Speed (Actual): " , leftDriveMotors[0].getSpeed());
		SmartDashboard.putNumber("Right Speed (Actual): " , rightDriveMotors[0].getSpeed());

	    	//System.out.println("speedChangeMultiplier" + speedChangeMultiplier);
    	//if(offset==0){
    		dSpeed = speed*(offset + speedChangeMultiplier); //speed=400, 0-->90; +3*400=1200
    	/*}else{
    		System.out.println("PID SpeedChangeMultiplier: " + speedChangeMultiplier + ", Angle: " + angle +", currentAngle: " + currentAngle);
    		dSpeed = speed*(offset); //speed=400, 0-->90; +3*400=1200
    	}*/
	    	/*if(Math.abs(dSpeed)>0.7*(Math.abs(speed))){
	    		dSpeed = Math.signum(dSpeed)*0.7*Math.abs(speed);
	    	}*/
		}
		SmartDashboard.putNumber("speedChangeMultiplier", speedChangeMultiplier);
		SmartDashboard.putNumber("Heading-DriveStraight", Constants.navX.getYaw());
		leftSpeed =-1*speed-Math.signum(speed)*dSpeed; //-1*400-1200 = -1800
		rightSpeed = speed-Math.signum(speed)*dSpeed; //400-1200 = -800
		System.out.println("Left: " + leftSpeed + ", Right: " + rightSpeed + ", dSpeed: " + dSpeed);
		
		SmartDashboard.putNumber("Left Speed (target)", leftSpeed);
		SmartDashboard.putNumber("Right Speed (target)", rightSpeed);
		
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
		leftDriveMotors[0].setPosition(0);
		rightDriveMotors[0].setPosition(0);
		
	}
	
	public void enableBrake(boolean enable){
		leftDriveMotors[0].enableBrakeMode(enable);
		rightDriveMotors[0].enableBrakeMode(enable);
	}
	
	

}
