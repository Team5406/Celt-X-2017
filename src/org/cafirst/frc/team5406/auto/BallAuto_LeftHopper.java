package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class BallAuto_LeftHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean done_before = false;
	private boolean readyToShoot = false;
	


	public BallAuto_LeftHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("BallAuto_LeftHopper");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
		robotShooter = _robotShooter;
	
	}
	
	public void init(){
		robotDrive.AutoInit();
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		autoStep =0;
		robotDrive.enableBrake(true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
		robotDrive.AutoInit();
		robotShooter.alignTurret();
		robotShooter.Shoot();
		robotShooter.BallPump(-1);
    	//robotIntake.IntakeBalls(0.5);
		
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
				turretInit = robotShooter.turnTurretToDegree(Constants.PRACTICE_BOT_LEFT_HOPPER_TURRET_START);
			}else{
				turretInit = robotShooter.turnTurretToDegree(Constants.COMP_BOT_LEFT_HOPPER_TURRET_START);
			}
		}
		if (robotDrive.AutoLoop()==2){
			if(!done_before && turretInit ==1){
				robotShooter.alignTurret();
				done_before = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot){
				robotShooter.getDistance();
				readyToShoot = true;
			}else if (readyToShoot){
				robotShooter.getDistance();
				robotShooter.Shoot();
				robotShooter.Indexer(865);				
			}
		}
	}
}
