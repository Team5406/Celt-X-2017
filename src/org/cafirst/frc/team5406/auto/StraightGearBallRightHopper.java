package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class StraightGearBallRightHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean done_before = false;
	private boolean readyToShoot = false;
	


	public StraightGearBallRightHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("11 - Middle Gear and Right Balls");
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
		robotShooter.alignTurret();
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
			if(Constants.IS_PRACTICE_BOT){
				turretInit = robotShooter.turnTurretToDegree(-70);
			}else{
				turretInit = robotShooter.turnTurretToDegree(-180);
			}
			System.out.println("TurretInit: " + turretInit);
		}
		
		
		
		switch (autoStep){
		case 0:
			
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[1]);
			if( direction*robotPosition[1] > ((111.5-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 1;
				//robotDrive.DriveStraight(0.2);
				robotIntake.dropGear(false);
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 0.0, true);
			}else if ( direction*robotPosition[1] > ((90-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(110, 0.0, true);
			}
			break;
		case 1:
			if (!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 0.0, false);
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    	            	robotDrive.driveAtAngleUpdate(-200, 0.0, false);
		    	            	//Do we need a this.cancel();?
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-300, 0.0, false);
			System.out.println("robotPosition (2) " + direction*robotPosition[1]);
			if(direction*robotPosition[1] < (-12/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 3;
            	robotDrive.resetPosition();
				robotIntake.liftGear();
				robotIntake.StopIntake();
			}
			break;
		case 3:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (3) " + direction*robotPosition[1]);
			if( Math.abs(direction*robotPosition[1]) > (108/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, -90, true);
				robotIntake.IntakeBalls(50);
				autoStep = 4;
			} else if( Math.abs(direction*robotPosition[1]) <= (108/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(-300, -90, true);
			}
			break;
		case 4:
			System.out.println("done_before (4) " + done_before);
			System.out.println("!robotShooter.centeringInProgress " + !robotShooter.centeringInProgress);
			System.out.println("readyToShoot " + readyToShoot);

			robotDrive.driveAtAngleEnd();
			robotDrive.enableBrake(false);
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
			break;
		
		}
	}
}

