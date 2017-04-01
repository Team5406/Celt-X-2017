package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


public class Drive extends Subsystems{
	private CANTalon[] leftDriveMotors;
	private CANTalon[] rightDriveMotors;
    private DoubleSolenoid shiftSolenoid;
    private PowerDistributionPanel pdp;
    public boolean precisionDrive = false;
    private boolean highGear = false;

	RobotDrive drive;
	public Drive(){
		leftDriveMotors = InitializeMotors(Constants.LEFT_DRIVE);
		rightDriveMotors = InitializeMotors(Constants.RIGHT_DRIVE);
		leftDriveMotors[0].setVoltageRampRate(100);
		rightDriveMotors[0].setVoltageRampRate(100);
		shiftSolenoid = new DoubleSolenoid(Constants.SHIFT_FORWARD, Constants.SHIFT_REVERSE);
		drive = new RobotDrive(leftDriveMotors[0], rightDriveMotors[0]);
		pdp = new PowerDistributionPanel();
		
	}
	
	public void ArcadeDrive(double x, double y){
		System.out.println("Climb function");
		leftDriveMotors[0].enable();
		rightDriveMotors[0].enable();
		int x_sign = 1;
		int y_sign = 1;
		if (precisionDrive){
			//x_sign = (int)(x/Math.abs(x));
			//y_sign = (int)(y/Math.abs(y));
			/*if (Math.abs(x)<Math.abs(y)){
				x = x_sign*y*Math.sqrt(Math.abs(x)/Math.abs(y));
			}else{
				y = y_sign*x*Math.sqrt(Math.abs(y)/Math.abs(x));
			}*/
			/*if (Math.abs(x)<Math.abs(y)){
				x = x_sign*y*Math.pow((x/y),4);
			}else{
				y = y_sign*x*Math.pow((y/x),4);
			}*/
			y *=0.5;//Math.signum(y) * Math.abs(Math.pow(y, 3));
			//x*=0.5;
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

}
