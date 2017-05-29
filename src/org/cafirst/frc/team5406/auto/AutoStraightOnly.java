package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.util.Constants;

public class AutoStraightOnly extends AutonomousRoutine{
	
	private Drive robotDrive;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	
	/**
	 * Constructor for Straight Only Auto
	 * @param robotDrive The Robot Drive System
	 */
	public AutoStraightOnly(Drive robotDrive)
	{
		super("10 - Straight Only");
		
		this.robotDrive = robotDrive;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		
		robotDrive.toggleBrakeMode(true);
		robotDrive.driveAtAngleStart(300, 0.0, true);
	}
	
	@Override
	public void periodic()
	{
		switch (autoStep)
		{
		case 0:
			robotPosition = robotDrive.getPosition();
			
			if(direction * robotPosition[1] > ((110.5 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 0.0, true);
				
				autoStep = 1;
			}
			break;
		}
	}
	
	@Override
	public void end()
	{
		robotDrive.toggleBrakeMode(false);
		robotDrive.driveAtAngleEnd();
	}
	
	@Override
	public AutonomousRoutine newInstance() {
		return new AutoStraightOnly(robotDrive);
	}
}
