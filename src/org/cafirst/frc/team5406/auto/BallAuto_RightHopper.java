package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class BallAuto_RightHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean turretAligned = false;
	private boolean readyToShoot = false;
	private double rpm = 5700;
	


	public BallAuto_RightHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("3 - Balls Only - Right Hopper");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
		robotShooter = _robotShooter;
	
	}
	
	public void init(){
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		autoStep =0;
		gearDelay = false;
		robotDrive.enableBrake(true);
		robotDrive.driveAtAngleInit(400, 0.0, true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
		//robotShooter.alignTurret();
		robotShooter.Shoot();
		robotShooter.BallPump(-1);
	}
	
	public void end(){
		robotShooter.stopTimer();
		robotDrive.enableBrake(false);
	}

	public void periodic(){
		System.out.println("Auto Step (Straight Gear): " + autoStep);
		double turretInit = 0;
		if (robotShooter.findTurretREVLimit() && turretInit == 0){
				turretInit = robotShooter.turnTurretToDegree(-70);
			
		}
		
		
		
		switch (autoStep){

		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[0]);
			if( direction*robotPosition[0] > ((113-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, 90.0, true);
				autoStep = 1;
			} else if( direction*robotPosition[0] > ((93-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, 90.0, true);
			}else if ( direction*robotPosition[0] > ((75-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, 45.0, true);
				robotIntake.IntakeBalls(50);
			}
			break;
		case 1:
			robotDrive.driveAtAngleEnd();
			if(!turretAligned && turretInit != 0){
				robotShooter.alignTurret();
				turretAligned = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot && turretAligned){
				robotShooter.getDistance();
				rpm = robotShooter.getRPM();
				System.out.println("TopRight: " + robotShooter.getImageTop() + ", RPM: " + rpm);
				readyToShoot = true;
				
			}else if (readyToShoot){
				//robotShooter.getRPM();
				//robotShooter.getDistance();
				robotShooter.Shoot(rpm);
				robotShooter.Indexer(Constants.INDEXER_SPEED);				
			}
			break;
		
		}
	}
}

