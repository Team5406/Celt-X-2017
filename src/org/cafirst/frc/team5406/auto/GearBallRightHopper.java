package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class GearBallRightHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean done_before = false;
	private boolean readyToShoot = false;
	


	public GearBallRightHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("GearBall_RightHopper");
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
		robotDrive.driveAtAngleInit(200, 0.0, true);
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
			System.out.println("robotPosition (0) " + direction*robotPosition[1]);
			if( direction*robotPosition[1] > ((150-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 1;
				robotIntake.dropGear(false);
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, -30.0, true);
			} else if( direction*robotPosition[1] > ((120-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(110, -30, true);
			}else if ( direction*robotPosition[1] > ((82-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(300, -30, true);
			}
			break;
		case 1:
			if (!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, -30.0, false);
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    	            	robotDrive.driveAtAngleUpdate(-200, -30.0, false);
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-200, -30.0, false);
			System.out.println("robotPosition (2) " + direction*robotPosition[1]);
			if(direction*robotPosition[1] < (-24/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 3;
				robotIntake.liftGear();
				robotIntake.StopIntake();
				robotDrive.driveAtAngleUpdate(200, 45, false);
				robotDrive.enableBrake(false);
				robotDrive.driveAtAngleEnd();
			}
			break;
		case 3:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (3) " + direction*robotPosition[0]);
			if( direction*robotPosition[0] > ((120-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, 90.0, true);
				autoStep = 4;
			} else if( direction*robotPosition[0] > ((70-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(300, 90.0, true);
			}
			break;
		case 4:
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

