package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.auto.AutonomousRoutine;



public class AutoLeftGearLeftBoiler  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	

	public AutoLeftGearLeftBoiler(Drive _robotDrive, Intake _robotIntake){
		super("6 - Left Gear (Left Boiler)");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
	}
	
	public void init(){
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		autoStep =0;
		gearDelay = false;
		robotDrive.enableBrake(true);
		robotDrive.driveAtAngleInit(300, 0.0, true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
	}
	
	public void end(){
		robotDrive.driveAtAngleEnd();
	}

	public void periodic(){
		System.out.println("Auto Step (Straight Gear): " + autoStep);
		
		switch (autoStep){
		case 0:
			robotPosition = robotDrive.getPosition();
			System.out.println("robotPosition (0) " + direction*robotPosition[0]);
			if( direction*robotPosition[0] > ((160-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 1;
				robotIntake.dropGear(false);
				robotDrive.resetPosition();
				robotDrive.driveAtAngleUpdate(0.0, 60.0, true);
			} else if( direction*robotPosition[0] > ((135-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(115, 60, true);
			}else if ( direction*robotPosition[0] > ((90-Constants.ROBOT_LENGTH)/(Constants.WHEEL_DIAM*Math.PI))){
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
		    	            	robotDrive.driveAtAngleUpdate(-200, 60.0, false);
		    	            	this.cancel();
		    	            }
		    	        }, 
		    	        500 
		    	);
			}
			break;
		case 2:
			robotPosition = robotDrive.getPosition();
			robotDrive.driveAtAngleUpdate(-200, 60.0, false);
			System.out.println("robotPosition (2) " + direction*robotPosition[0]);
			if(direction*robotPosition[0] < (-24/(Constants.WHEEL_DIAM*Math.PI))){
				autoStep = 3;
				robotIntake.liftGear();
				robotIntake.StopIntake();
				robotDrive.enableBrake(false);
				robotDrive.driveAtAngleEnd();
			}
			break;
		}

	}
}
