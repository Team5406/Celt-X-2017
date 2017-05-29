package org.cafirst.frc.team5406.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.subsystem.Shooter;
import org.cafirst.frc.team5406.util.Constants;

public class BallAutoLeftHopper extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Intake robotIntake;
	private Shooter robotShooter;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	
	private boolean gearDelay;
	
	private double turretStatus = 0;
	private double rpm = 5900;
	
	private boolean isTurretAligned = false;
	private boolean isReadyToShoot = false;
	
	/**
	 * Constructor for Left Ball Hopper Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotShooter The Robot Shooter System
	 * @param robotIntake The Robot Intake System
	 */
	public BallAutoLeftHopper(Drive robotDrive, Shooter robotShooter, Intake robotIntake)
	{
		super("2 - Balls Only - Left Hopper");
		
		this.robotDrive = robotDrive;
		this.robotIntake = robotIntake;
		this.robotShooter = robotShooter;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
		gearDelay = false;
		
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		
		robotDrive.toggleBrakeMode(false);
		robotDrive.driveAtAngleStart(400, 0.0, true);
		
		turretStatus = 0;
		robotShooter.shoot();
		robotShooter.setBallPump(-1);
	}
	
	@Override
	public void periodic()
	{
		if(robotShooter.findRevLimit() && turretStatus == 0)
			turretStatus = robotShooter.turnTurret(-262);
		
		switch(autoStep)
		{
		case 0:
			robotPosition = robotDrive.getPosition();
			
			if(direction * robotPosition[1] > ((114 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(0.0, -90.0, true);
            	robotDrive.resetPosition();
				autoStep = 1;
			} 
			else if(direction * robotPosition[1] > ((109 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(200, -90.0, true);
			else if(direction * robotPosition[1] > ((94 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(400, -90.0, true);
			else if(direction * robotPosition[1] > ((73 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(400, -45.0, true);
				robotIntake.intakeBalls(50);
			}
			break;
			
		case 1:
			if (!gearDelay){
				gearDelay = true;
		    	new Timer().schedule( 
		    			new TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    					this.cancel();
		    	            }
		    	        }, 
		    	        1000 
		    	);
			}
			break;
			
		case 2:
			robotPosition = robotDrive.getPosition();
			
			robotDrive.driveAtAngleUpdate(0, 0.0, true);
			robotDrive.driveAtAngleEnd();
			
			if(!isTurretAligned && turretStatus != 0){
				robotShooter.alignTurret();
				isTurretAligned = true;
			}else if(!robotShooter.isVisionCentering() && !isReadyToShoot && isTurretAligned){
				rpm = robotShooter.getRPM();
				isReadyToShoot = true;
				
				System.out.println("TopRight: " + robotShooter.getImageTop() + ", RPM: " + rpm);
			}
			
			if(isReadyToShoot)
			{
				robotShooter.shoot(rpm);
				robotShooter.setIndexer(Constants.indexer.getTarget());				
			}
			break;
		}
	}
	
	@Override
	public void end()
	{
		robotShooter.stopTimer();
		robotDrive.toggleBrakeMode(false);
	}
	
	@Override
	public AutonomousRoutine newInstance() {
		return new BallAutoLeftHopper(robotDrive, robotShooter, robotIntake);
	}
}
