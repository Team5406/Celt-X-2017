package org.cafirst.frc.team5406.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.util.Constants;

public class AutoMiddleGear extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Intake robotIntake;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	
	private boolean gearDelay;
	
	/**
	 * Constructor for Middle Gear Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotIntake The Robot Intake System
	 */
	public AutoMiddleGear(Drive robotDrive, Intake robotIntake)
	{
		super("1 - Middle Gear");
		
		this.robotDrive = robotDrive;
		this.robotIntake = robotIntake;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
		gearDelay = false;
		
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		
		robotDrive.toggleBrakeMode(true);
		robotDrive.driveAtAngleStart(300, 0.0, true);
	}
	
	@Override
	public void periodic()
	{	
		switch(autoStep)
		{
		case 0:
			robotPosition = robotDrive.getPosition();
			
			if(direction * robotPosition[1] > ((110.5 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotIntake.dropGear(false);
				
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 0.0, true);
				
				autoStep = 1;
			}
			else if(direction * robotPosition[1] > ((90 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(110, 0.0, true);
				
			break;
			
		case 1:
			if(!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 0.0, false);
		    	new Timer().schedule( 
		    			new TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    	            	robotDrive.driveAtAngleUpdate(-200, 0.0, false);
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
			
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-200, 0.0, false);
			
			if(direction*robotPosition[1] < (-24/(Constants.WHEEL_DIAM*Math.PI)))
			{
				robotIntake.liftGear();
				robotIntake.stopIntake();
				robotDrive.driveAtAngleEnd();
				
				autoStep = 3;
			}
			break;
		}
	}
	
	@Override
	public void end()
	{
		robotDrive.driveAtAngleEnd();
		robotDrive.toggleBrakeMode(false);
	}
	
	@Override
	public AutonomousRoutine newInstance() {
		return new AutoMiddleGear(robotDrive, robotIntake);
	}
}
