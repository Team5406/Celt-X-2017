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
	double turretInit = 0;

	


	public BallAuto_RightHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("3 - Balls Only - Right Hopper");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
		robotShooter = _robotShooter;
		robotShooter.cameraOffset = Constants.CAMERA_RIGHT_OFFSET;

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
		turretInit = 0;
		robotShooter.cameraOffset = Constants.CAMERA_RIGHT_OFFSET;

	}
	
	public void end(){
		robotShooter.stopTimer();
		robotDrive.driveAtAngleEnd();
		robotDrive.enableBrake(false);
	}

	public void periodic(){
		System.out.println("Auto Step (Straight Gear): " + autoStep);
		if (robotShooter.findTurretREVLimit() && turretInit == 0){
				turretInit = robotShooter.turnTurretToDegree(-66);
			
		}
		
		
		
		switch (autoStep){

		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[0]);
			if( direction*robotPosition[0] > ((127-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(110, 90.0, true);
				autoStep = 1;
			} else if( direction*robotPosition[0] > ((112-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(200, 90.0, true);
			} else if( direction*robotPosition[0] > ((96-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
						robotDrive.driveAtAngleUpdate(400, 90.0, true);
			}else if ( direction*robotPosition[0] > ((75-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, 45.0, true);
				robotShooter.Shoot();
				robotShooter.BallPump(-1);
				robotIntake.IntakeBalls(50);
			}
			break;
		case 1:
			if (!gearDelay){
				gearDelay = true;
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 2;
		    	            	robotDrive.resetPosition();
		    					//robotDrive.driveAtAngleUpdate(-110, 0.0, true);
		    					this.cancel();
		    	            }
		    	        }, 
		    	        350 
		    	);
			}
			
			break;
		case 2:
			robotDrive.driveAtAngleEnd();
			if(!turretAligned && turretInit != 0){
				robotShooter.alignTurret();
				turretAligned = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot && turretAligned){
				//robotShooter.getDistance();
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

