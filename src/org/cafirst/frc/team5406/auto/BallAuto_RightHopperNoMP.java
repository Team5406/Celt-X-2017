package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class BallAuto_RightHopperNoMP  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean turretAligned = false;
	private boolean readyToShoot = false;
	


	public BallAuto_RightHopperNoMP(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
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
		if (robotShooter.findTurretCCWLimit() && turretInit == 0){
			if(Constants.IS_PRACTICE_BOT){
				turretInit = robotShooter.turnTurretToDegree(Constants.PRACTICE_BOT_RIGHT_HOPPER_TURRET_START);
			}else{
				turretInit = robotShooter.turnTurretToDegree(Constants.COMP_BOT_RIGHT_HOPPER_TURRET_START);
			}
		}
		
		
		
		switch (autoStep){

		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[0]);
			if( direction*robotPosition[0] > ((184-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, 90.0, true);
				autoStep = 1;
			} else if( direction*robotPosition[0] > ((124-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, 90.0, true);
			}else if ( direction*robotPosition[0] > ((86-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, 45.0, true);
			}
			break;
		case 1:
			robotDrive.driveAtAngleEnd();
			if(!turretAligned && turretInit != 0){
				robotShooter.alignTurret();
				turretAligned = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot){
				robotShooter.getDistance();
				readyToShoot = true;
			}else if (readyToShoot){
				robotShooter.getDistance();
				robotShooter.Shoot();
				robotShooter.Indexer(865);				
			}
			break;
		
		}
	}
}
