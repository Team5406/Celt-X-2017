package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.auto.AutonomousRoutine;



public class AutoStraightOnly  extends AutonomousRoutine{
	private Drive robotDrive;
	private int autoStep = 0;
	private double[] robotPosition;
	private int direction = 1;
	

	public AutoStraightOnly(Drive _robotDrive){
		super("AutoStraightOnly");
		robotDrive = _robotDrive;
	}
	
	public void init(){
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		autoStep =0;
		robotDrive.enableBrake(true);
		robotDrive.driveAtAngleInit(300, 0.0, true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
	}
	
	public void end(){
		robotDrive.driveAtAngleEnd();
	}

	public void periodic(){
		System.out.println("Auto Step (Straight Only): " + autoStep);
		
		switch (autoStep){
		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[1]);
			if( direction*robotPosition[1] > ((110.5-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 1;
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 0.0, true);
			}
			break;
		}
	}
}
