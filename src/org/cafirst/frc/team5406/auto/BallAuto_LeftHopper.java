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
	private boolean turretAligned = false;
	private boolean readyToShoot = false;
	


	public BallAuto_LeftHopper(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		super("2 - Balls Only - Left Hopper");
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
			turretInit = robotShooter.turnTurretToDegree(-270);

		}
		
		
		
		switch (autoStep){

		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[1]);
			if( direction*robotPosition[1] > ((115-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0, -90.0, true);
            	robotDrive.resetPosition();
				autoStep = 1;
			} else if( direction*robotPosition[1] > ((95-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, -90.0, true);
			}else if ( direction*robotPosition[1] > ((74-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(400, -45.0, true);
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
		    	        1000 
		    	);
			}
			
			break;
		case 2:
			robotPosition = robotDrive.getPosition();
			System.out.println(direction*robotPosition[1]); 
			System.out.println(10/(Constants.WHEEL_DIAM*Math.PI));
			//9.4814453125
			//3.9788735772973833

			System.out.println(System.nanoTime() + " Robot Position" + direction*robotPosition[1]);

	
			/*//Drive backward and straighten to compensate for turret angle
			 * if( Math.abs(direction*robotPosition[1]) < (12/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(-400, 0.0, true);
				System.out.println(System.nanoTime() + " Drive at Angle");
			}else{*/
			robotDrive.driveAtAngleUpdate(0, 0.0, true);
			robotDrive.driveAtAngleEnd();
			System.out.println(System.nanoTime() + " Drive at Angle End");
			if(!turretAligned && turretInit != 0){
				robotShooter.alignTurret();
				System.out.println(System.nanoTime() + " Aligning Turret");
				turretAligned = true;
			}else if (!robotShooter.centeringInProgress && !readyToShoot && turretAligned){
				robotShooter.getDistance();
				System.out.println(System.nanoTime() + " Getting Distance");

				readyToShoot = true;
			}
			
			if (readyToShoot){
				System.out.println(System.nanoTime() + " Shooting");
				robotShooter.getDistance();
				robotShooter.Shoot();
				robotShooter.Indexer(865);				
			}
			//}
			break;
		
		}
	}
}

