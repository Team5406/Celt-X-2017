package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.util.Constants;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;

public class AutoLeftGearLeftBoiler extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Intake robotIntake;
	
	private int autoStep;
	
	private int direction;
	
	private double[] robotPosition;
	private double[] startPosition;
	private double[] robotDistances = {0, 0};
	private double[] robotAngles = {0,0,0,0};
	
	private boolean gearDelay;
	
	/**
	 * Constructor for Left Gear Left Boiler Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotIntake The Robot Intake System
	 */
	public AutoLeftGearLeftBoiler(Drive robotDrive, Intake robotIntake)
	{
		super("6 - Left Gear (Left Boiler)");
		
		this.robotDrive = robotDrive;
		this.robotIntake = robotIntake;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
		gearDelay = false;
		
		startPosition = robotDrive.getPosition();
		
		Constants.navX.zeroYaw();
		robotAngles[0] = Constants.navX.getYaw();
		
		robotDrive.driveAtAngleStart(250, 0.0, true);
	}
	
	@Override
	public void periodic()
	{
		robotPosition = robotDrive.getPosition();
		
		robotDistances[0] = startPosition[0] - robotPosition[0];
		robotDistances[1] = startPosition[1] - robotPosition[1];
		
		switch(autoStep)
		{
		case 0:
			if( Math.abs(direction * robotDistances[1]) > ((35.5) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtCurveStart(300, 48, 55, 4);
				robotAngles[1] = Constants.navX.getYaw();
				
				autoStep = 1;
			}
			break;
			
		case 1:
			if(robotDrive.isCurveCompleted())
			{
				robotDrive.driveAtCurveEnd();
				robotDrive.driveAtAngleUpdate(300, 60, true);
				
				startPosition = robotPosition;
				robotAngles[2] = Constants.navX.getYaw();
				
				autoStep = 2;
			}
			break;
			
		case 2:
			if(Math.abs(direction * robotDistances[1]) > ((34) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotIntake.dropGear(false);
				robotDrive.driveAtAngleUpdate(0, 60, true);
				
				robotAngles[3] = Constants.navX.getYaw();
				
				autoStep = 3;
			}
			else if(Math.abs(direction * robotDistances[1]) > ((22) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(150, 60, true);
			
			break;
			
		case 3:
			if(!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 60.0, false);
		    	new Timer().schedule( 
		    			new TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 4;
		    	            	startPosition = robotPosition;
		    	            	robotDrive.driveAtAngleUpdate(-200, 60.0, true);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
			
		case 4:
			robotDrive.driveAtAngleUpdate(-200, 60.0, true);
			if(Math.abs(direction * robotDistances[1]) > (24 / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotIntake.liftGear();
				robotIntake.stopIntake();
				robotDrive.driveAtAngleEnd();
				
				autoStep = 5;
			}
			break;
		}
	}
	
	@Override
	public void end()
	{
		robotDrive.toggleBrakeMode(false);
		robotDrive.driveAtCurveEnd();
		robotDrive.driveAtAngleEnd();
	}

	@Override
	public AutonomousRoutine newInstance() {
		return new AutoLeftGearLeftBoiler(robotDrive, robotIntake);
	}
}
