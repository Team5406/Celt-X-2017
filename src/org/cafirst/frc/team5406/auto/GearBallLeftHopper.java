package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class GearBallLeftHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean done_before = false;
	private boolean readyToShoot = false;
	


	public GearBallLeftHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("4 - Left Gear and Left Hopper");
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
	}
	
	public void end(){
		robotShooter.stopTimer();
		robotDrive.enableBrake(false);
	}

	public void periodic(){
		System.out.println("Auto Step (Straight Gear): " + autoStep);
		double turretInit = 0;
		if (robotShooter.findTurretREVLimit() && turretInit == 0){
			turretInit = robotShooter.turnTurretToDegree(-270);
		}
		
		
		
		switch (autoStep){
		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[1]);
			if( direction*robotPosition[0] > ((152-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 1;
				robotIntake.dropGear(false);
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 60.0, true);
			} else if( direction*robotPosition[0] > ((120-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(200, 60, true);
			}else if ( direction*robotPosition[0] > ((75-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(300, 60, true);
			}
			break;
			
		case 1:
			if (!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 60.0, false);
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    	            	robotDrive.driveAtAngleUpdate(-500, 60.0, true);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-500, 60.0, true);
			System.out.println("robotPosition (2) " + direction*robotPosition[1]);
			if(Math.abs(direction*robotPosition[1]) < (12/(Constants.WHEEL_DIAM*Math.PI))){
				robotShooter.Shoot();
				robotShooter.BallPump(-1);
				robotIntake.IntakeBalls(50);
				robotDrive.driveAtAngleUpdate(-500, 0.0, true);
				autoStep = 3;
				robotIntake.liftGear();
				robotDrive.resetPosition();
			}
			break;
		
		case 3:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (3) 0:" + direction*robotPosition[0] + " 1:" + direction*robotPosition[1]);
			if( Math.abs(direction*robotPosition[1]) > ((55)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(500, -45.0, true);
				robotDrive.resetPosition();
				autoStep = 4;
			}
			break;
		case 4:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (4) 0:" + direction*robotPosition[0] + " 1:" + direction*robotPosition[1]);
			if( Math.abs(direction*robotPosition[0]) > ((90)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, -90.0, true);
				autoStep = 5;
			} else if( Math.abs(direction*robotPosition[0]) > ((20)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(500, -90.0, true);
			} 
			break;
		case 5:
			robotDrive.driveAtAngleEnd();
			robotDrive.enableBrake(false);
			if(!done_before && turretInit ==1){
				robotShooter.alignTurret();
				done_before = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot && done_before){
				robotShooter.getDistance();
				readyToShoot = true;
			}else if (readyToShoot){
				robotShooter.getDistance();
				robotShooter.Shoot();
				robotShooter.Indexer(Constants.INDEXER_SPEED);				
			}
			break;
		
		}
	}
}

