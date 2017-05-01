package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;



public class GearBallRightHopper  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private Shooter robotShooter;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private boolean turretAligned = false;
	private boolean done_before = false;
	private boolean readyToShoot = false;
	private double rpm = 5900;
	private double[] startPos = {0,0};
	private double[] robotDistance = {0,0};
	
	private double[] angles = {0,0,0,0};
	double turretInit = 0;



	public GearBallRightHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("5 - Right Gear and Right Hopper");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
		robotShooter = _robotShooter;

	}
	
	public void init(){
		Constants.navX.zeroYaw();
		startPos = robotDrive.getPosition();
		autoStep =0;
		gearDelay = false;
		robotDrive.driveAtAngleInit(250, 0.0, true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
		angles[0] = Constants.navX.getYaw();
		robotDrive.setVolageRampRate(75);
		turretInit = 0;
		
	}
	
	public void end(){
		robotDrive.enableBrake(false);
		robotDrive.driveAlongCurveEnd();
		robotDrive.driveAtAngleEnd();
		robotDrive.setVolageRampRate(100);

	}

	public void periodic(){
		System.out.println("Step: " + autoStep + " 0: " +  angles[0]+ " 1: " +  angles[1]+ " 2: " +  angles[2]+ " 3: " +  angles[3] + " current: " + Constants.navX.getYaw());
		if (robotShooter.findTurretREVLimit() && turretInit == 0){
			turretInit = robotShooter.turnTurretToDegree(-60);
		}
		
		robotPosition = robotDrive.getPosition();
		robotDistance[0] = startPos[0]-robotPosition[0];
		robotDistance[1] = startPos[1]-robotPosition[1];
		

		System.out.println("Left Pos: " + robotDistance[0] + ", Right Pos: " + robotDistance[1]);

		switch (autoStep){

		case 0:
			if( Math.abs(direction*robotDistance[1]) > ((35)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAlongCurveInit(300, 48, -55, 4);
				autoStep = 1;
				angles[1] = Constants.navX.getYaw();

			}
			break;
		case 1:
			if(robotDrive.driveAlongCurveCompleted()){
				robotDrive.driveAlongCurveEnd();
				robotDrive.driveAtAngleUpdate(300, -60, true);

				startPos = robotPosition;
				autoStep = 2;
				angles[2] = Constants.navX.getYaw();
			}
			break;
		case 2:
			if( Math.abs(direction*robotDistance[1]) > ((46)/(Constants.WHEEL_DIAM*Math.PI))){
				robotIntake.dropGear(false);
				robotDrive.driveAtAngleUpdate(0, -60, true);
				autoStep = 3;
				angles[3] = Constants.navX.getYaw();
			} else if( Math.abs(direction*robotDistance[1]) > ((30)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(150, -60, true);
			}
			break;
		case 3:
			if (!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, -60.0, true);
				robotDrive.resetPosition();
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 4;
		    	            	startPos = robotPosition;
		    	            	robotDrive.driveAtAngleUpdate(-400, -60.0, true);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 4:
			robotDrive.driveAtAngleUpdate(-200, -60.0, true);
			if(Math.abs(direction*robotDistance[1]) > (15/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 5;
				robotIntake.liftGear();
				robotDrive.driveAlongCurveInit(-400, 30, -5, 4);
			}
			break;
		case 5:
			if(robotDrive.driveAlongCurveCompleted()){
				robotDrive.driveAlongCurveEnd();
            	robotDrive.driveAtAngleUpdate(-400, 0.0, true);
            	startPos = robotPosition;
				autoStep = 6;
			}
			break;
		case 6:
			if(Math.abs(direction*robotDistance[1]) > (11/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 7;
				robotDrive.driveAlongCurveInit(400, 30, 80, 4);
			}else if(Math.abs(direction*robotDistance[1]) > (8/(Constants.WHEEL_DIAM*Math.PI))){
            	robotDrive.driveAtAngleUpdate(-100, 0.0, true);
			}
			break;
		case 7:
			if(robotDrive.driveAlongCurveCompleted()){
				robotDrive.driveAlongCurveEnd();
            	robotDrive.driveAtAngleUpdate(400, 90.0, true);
				robotShooter.Shoot();
				robotShooter.BallPump(-1);
				robotIntake.IntakeBalls(50);
            	startPos = robotPosition;
				autoStep = 8;
			}
			break;
		case 8:
			if(Math.abs(direction*robotDistance[1]) > (34/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 9;
            	robotDrive.driveAtAngleUpdate(0.0, 90.0, true);
            	robotDrive.driveAtAngleEnd();
			}else if(Math.abs(direction*robotDistance[1]) > (27/(Constants.WHEEL_DIAM*Math.PI))){
            	robotDrive.driveAtAngleUpdate(200, 90.0, true);
			}

			break;
		case 9:
			robotDrive.driveAtAngleEnd();
			if(!done_before && turretInit ==1){
				robotShooter.alignTurret();
				done_before = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot && done_before){
				//robotShooter.getDistance();
				readyToShoot = true;
			}else if (readyToShoot){
				//robotShooter.getDistance();
				robotShooter.Shoot();
				robotShooter.Indexer(Constants.INDEXER_SPEED);				
			}
			break;

		}
		
	}
}

