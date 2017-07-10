package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;



public class AutoLeftGearRightBoiler  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	private double[] startPos = {0,0};
	private double[] robotDistance = {0,0};
	
	private double[] angles = {0,0,0,0};
	double turretInit = 0;




	public AutoLeftGearRightBoiler(Drive _robotDrive, Intake _robotIntake){
		super("8 - Left Gear (Right Boiler)");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;

	}
	
	public void init(){
		Constants.navX.zeroYaw();
		startPos = robotDrive.getPosition();
		autoStep =0;
		gearDelay = false;
		robotDrive.driveAtAngleInit(200, 0.0, true);
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

		SmartDashboard.putNumber("DriveAngle", Constants.navX.getYaw());

		
		robotPosition = robotDrive.getPosition();
		robotDistance[0] = startPos[0]-robotPosition[0];
		robotDistance[1] = startPos[1]-robotPosition[1];
		

		System.out.println("Left Pos: " + robotDistance[0] + ", Right Pos: " + robotDistance[1]);

		switch (autoStep){

		case 0:
			if( Math.abs(direction*robotDistance[1]) > ((22)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAlongCurveInit(200, 60, 40, 2);
				autoStep = 1;
				angles[1] = Constants.navX.getYaw();

			}
			break;
		case 1:
			if(robotDrive.driveAlongCurveCompleted()){
				robotDrive.driveAlongCurveEnd();
				robotDrive.driveAlongCurveInit(200, 60, 58, 0);
				autoStep = 2;
			}
			break;
		case 2:
			if(robotDrive.driveAlongCurveCompleted()){
				robotDrive.driveAlongCurveEnd();
				robotDrive.driveAtAngleUpdate(200, 60, true);

				startPos = robotPosition;
				autoStep = 3;
				angles[2] = Constants.navX.getYaw();
			}
			break;
		case 3:
			if( Math.abs(direction*robotDistance[1]) > ((20)/(Constants.WHEEL_DIAM*Math.PI))){
				robotIntake.dropGear(false);
				robotDrive.driveAtAngleUpdate(0, 60, true);
				autoStep = 4;
				angles[3] = Constants.navX.getYaw();
			} else if( Math.abs(direction*robotDistance[1]) > ((10)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(200, 60, true);
			}
			break;
		case 4:
			if (!gearDelay){
				gearDelay = true;
				robotDrive.driveAtAngleUpdate(0.0, 60.0, true);
				robotDrive.resetPosition();
		    	new java.util.Timer().schedule( 
		    			new java.util.TimerTask() {
		    	            @Override
		    	            public void run() {
		    	            	autoStep = 5;
		    	            	startPos = robotPosition;
		    	            	robotDrive.driveAtAngleUpdate(-200, 60.0, true);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 5:
			robotDrive.driveAtAngleUpdate(-200, 60.0, true);
			if(Math.abs(direction*robotDistance[1]) > (18/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 6;
				robotIntake.liftGear();
				robotDrive.driveAtAngleUpdate(0, 60, true);
			}
			break;
		}
		
	}
}

