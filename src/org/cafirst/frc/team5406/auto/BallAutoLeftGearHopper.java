package org.cafirst.frc.team5406.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.subsystem.Shooter;
import org.cafirst.frc.team5406.util.Constants;

public class BallAutoLeftGearHopper extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Shooter robotShooter;
	private Intake robotIntake;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	private double[] startPosition;
	private double[] robotDistance = {0, 0};
	private double[] robotAngles = {0, 0, 0, 0};
	
	private boolean gearDelay = false;
	
	private double turretStatus = 0;
	private boolean isTurretAligned = false;
	private boolean isReadyToShoot = false;
	
	/**
	 * Constructor for Left Gear and Hopper Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotShooter The Robot Shooter System
	 * @param robotIntake The Robot Intake System
	 */
	public BallAutoLeftGearHopper(Drive robotDrive, Shooter robotShooter, Intake robotIntake)
	{
		super("4 - Left Gear and Left Hopper");
		
		this.robotDrive = robotDrive;
		this.robotIntake = robotIntake;
		this.robotShooter = robotShooter;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
		turretStatus = 0;
		
		gearDelay = false;
		
		Constants.navX.zeroYaw();
		robotDrive.driveAtAngleStart(250, 0.0, true);
		robotDrive.setVoltageRampRate(75);
		
		startPosition = robotDrive.getPosition();
		robotAngles[0] = Constants.navX.getYaw();
	}
	
	@Override
	public void periodic()
	{
		if(robotShooter.findRevLimit() && turretStatus == 0)
			turretStatus = robotShooter.turnTurret(-262);
		
		robotPosition = robotDrive.getPosition();
		robotDistance[0] = startPosition[0]-robotPosition[0];
		robotDistance[1] = startPosition[1]-robotPosition[1];
		
		switch(autoStep)
		{
		case 0:
			if(Math.abs(direction * robotDistance[1]) > ((28.5) / (Constants.WHEEL_DIAM * Math.PI)))
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
			if(Math.abs(direction * robotDistance[1]) > ((55) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotIntake.dropGear(false);
				robotDrive.driveAtAngleUpdate(0, 60, true);
				
				robotAngles[3] = Constants.navX.getYaw();
				
				autoStep = 3;
			} 
			else if(Math.abs(direction * robotDistance[1]) > ((39) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(150, 60, true);
			
			break;
			
		case 3:
			if(!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 60.0, true);
				robotDrive.resetPosition();
		    	new Timer().schedule( 
		    			new TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 4;
		    	            	startPosition = robotPosition;
		    	            	robotDrive.driveAtAngleUpdate(-400, 60.0, true);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
			
		case 4:
			robotDrive.driveAtAngleUpdate(-200, 60.0, true);
			if(Math.abs(direction * robotDistance[1]) > (15 / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotIntake.liftGear();
				robotDrive.driveAtCurveStart(-400, 30, 5, 4);
				
				autoStep = 5;
			}
			break;
			
		case 5:
			if(robotDrive.isCurveCompleted()){
				robotDrive.driveAtCurveEnd();
            	robotDrive.driveAtAngleUpdate(-400, 0.0, true);
            	
            	startPosition = robotPosition;
				
            	autoStep = 6;
			}
			break;
			
		case 6:
			if(Math.abs(direction * robotDistance[1]) > (19 / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtCurveStart(400, 30, -80, 4);
				autoStep = 7;
			}
			else if(Math.abs(direction * robotDistance[1]) > (12 / (Constants.WHEEL_DIAM * Math.PI)))
            	robotDrive.driveAtAngleUpdate(-100, 0.0, true);
			
			break;
			
		case 7:
			if(robotDrive.isCurveCompleted())
			{
				robotDrive.driveAtCurveEnd();
            	robotDrive.driveAtAngleUpdate(400, -90.0, true);
				
            	robotShooter.shoot();
				robotShooter.setBallPump(-1);
				robotIntake.intakeBalls(50);
            	
				startPosition = robotPosition;
				
				autoStep = 8;
			}
			break;
			
		case 8:
			if(Math.abs(direction * robotDistance[1]) > (23 / (Constants.WHEEL_DIAM * Math.PI)))
			{
            	robotDrive.driveAtAngleUpdate(0.0, -90.0, true);
            	robotDrive.driveAtAngleEnd();
            	
				autoStep = 9;
			}
			else if(Math.abs(direction * robotDistance[1]) > (18 / (Constants.WHEEL_DIAM * Math.PI)))
            	robotDrive.driveAtAngleUpdate(200, -90.0, true);
			
			break;
			
		case 9:
			robotDrive.driveAtAngleEnd();
			if(!isTurretAligned && turretStatus == 1)
			{
				robotShooter.alignTurret();
				isTurretAligned = true;
			}
			else if(!robotShooter.isVisionCentering() && !isReadyToShoot && isTurretAligned)
				isReadyToShoot = true;
			else if(isReadyToShoot)
			{
				robotShooter.shoot();
				robotShooter.setIndexer(Constants.indexer.getTarget());				
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
		robotDrive.setVoltageRampRate(100);
	}
	
	@Override
	public AutonomousRoutine newInstance() {
		return new BallAutoLeftGearHopper(robotDrive, robotShooter, robotIntake);
	}
}
