package org.cafirst.frc.team5406.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.subsystem.Shooter;
import org.cafirst.frc.team5406.util.Constants;

public class BallAutoMiddleGearRightBoiler extends AutonomousRoutine{
	
	private Drive robotDrive;
	private Shooter robotShooter;
	private Intake robotIntake;
	
	private int autoStep;
	
	private int direction;
	private double[] robotPosition;
	
	private boolean gearDelay = false;
	
	private double turretStatus = 0;
	private double rpm = 5000;
	private boolean isTurretAligned = false;
	private boolean isReadyToShoot = false;
	
	/**
	 * Constructor for Middle Gear and Right Boiler Auto
	 * @param robotDrive The Robot Drive System
	 * @param robotShooter The Robot Shooter System
	 * @param robotIntake The Robot Intake System
	 */
	public BallAutoMiddleGearRightBoiler(Drive robotDrive, Shooter robotShooter, Intake robotIntake)
	{
		super("11 - Middle Gear and Right Balls");
		
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
		robotDrive.resetPosition();
		
		robotDrive.toggleBrakeMode(true);
		robotDrive.driveAtAngleStart(300, 0.0, true);
	}
	
	@Override
	public void periodic()
	{
		if(robotShooter.findRevLimit() && turretStatus == 0)
			turretStatus = robotShooter.turnTurret(-200);
		
		switch(autoStep)
		{
		case 0:	
			robotPosition = robotDrive.getPosition();
			
			if(direction * robotPosition[1] > ((113.5 - Constants.ROBOT_LENGTH) / (Constants.WHEEL_DIAM * Math.PI)))
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
		    	            	robotDrive.resetPosition();
		    	            	robotDrive.driveAtAngleUpdate(-200, 0.0, false);
		    	            	
		    	            	autoStep = 2;
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
			
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-300, 0.0, false);
			
			if(direction * robotPosition[1] < (-12 / (Constants.WHEEL_DIAM * Math.PI)))
			{
            	robotDrive.resetPosition();
				robotIntake.liftGear();
				robotIntake.stopIntake();
				
				autoStep = 3;
			}
			break;
			
		case 3:
			robotPosition = robotDrive.getPosition();
			
			if(Math.abs(direction * robotPosition[1]) > (103 / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(0.0, -90, true);
				robotIntake.intakeBalls(50);
				
				autoStep = 4;
			}else if(Math.abs(direction * robotPosition[1]) <= (103 / (Constants.WHEEL_DIAM * Math.PI)))
			{
				robotDrive.driveAtAngleUpdate(-300, -90, true);
				robotShooter.shoot();
				robotShooter.setBallPump(-1);
			}
			break;
			
		case 4:
			robotDrive.driveAtAngleEnd();
			robotDrive.toggleBrakeMode(false);
			
			if(!isTurretAligned && turretStatus == 1){
				robotShooter.alignTurret();
				isTurretAligned = true;
			}else if(!robotShooter.isVisionCentering() &&  !isReadyToShoot && isTurretAligned){
				rpm = robotShooter.getRPM();
				isReadyToShoot = true;
			}else if(isReadyToShoot){
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
		return new BallAutoMiddleGearRightBoiler(robotDrive, robotShooter, robotIntake);
	}
}
