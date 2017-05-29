package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.subsystem.Shooter;
import org.cafirst.frc.team5406.util.Constants;

public class BallAutoRightHopper extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Intake robotIntake;
	private Shooter robotShooter;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	
	private double turretStatus = 0;
	private double rpm = 5700;
	
	private boolean isTurretAligned = false;
	private boolean isReadyToShoot = false;
	
	/**
	 * Constructor for Left Ball Hopper Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotShooter The Robot Shooter System
	 * @param robotIntake The Robot Intake System
	 */
	public BallAutoRightHopper(Drive robotDrive, Shooter robotShooter, Intake robotIntake)
	{
		super("3 - Balls Only - Left Hopper");
		
		this.robotDrive = robotDrive;
		this.robotIntake = robotIntake;
		this.robotShooter = robotShooter;
	}
	
	@Override
	public void init()
	{
		autoStep = 0;
		direction = (Constants.IS_PRACTICE_BOT ? 1 : -1);
		
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
			
			if(direction * robotPosition[0] > ((117 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(0.0, 90.0, true);
				autoStep = 1;
			}
			else if(direction * robotPosition[0] > ((109 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(200, 90.0, true);
			else if(direction * robotPosition[0] > ((93 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
				robotDrive.driveAtAngleUpdate(400, 90.0, true);
			else if(direction * robotPosition[0] > ((75 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(400, 45.0, true);
				robotIntake.intakeBalls(50);
			}
			break;
			
		case 1:
			robotDrive.driveAtAngleEnd();
			
			if(!isTurretAligned && turretStatus != 0)
			{
				robotShooter.alignTurret();
				isTurretAligned = true;
			}
			else if(!robotShooter.isVisionCentering() && !isReadyToShoot && isTurretAligned)
			{
				rpm = robotShooter.getRPM();
				isReadyToShoot = true;
				
				System.out.println("TopRight: " + robotShooter.getImageTop() + ", RPM: " + rpm);
			}
			else if(isReadyToShoot)
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
		return new BallAutoRightHopper(robotDrive, robotShooter, robotIntake);
	}
}
