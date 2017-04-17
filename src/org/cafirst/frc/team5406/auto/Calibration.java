package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.PrintWriter;



public class Calibration{
	private Intake robotIntake;
	private Shooter robotShooter;
	private Drive robotDrive;
	private int increment = 0;
	private int increment_inches = 6;
	private double[] robotPosition;
	private int direction = 1;
	private double RPM = 4000;
	private boolean doneBefore = false;
	private double[] data = new double[4]; //encoder, imageY, centerX, RPM
	PrintWriter pw;
	StringBuilder dataSet;
	private boolean init = false;
	private boolean inProgress = false;
	private boolean newDrive = true;
	private double turretTarget = 0;
	
	
	
	public Calibration(Drive _robotDrive, Intake _robotIntake, Shooter _robotShooter){
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
		robotShooter = _robotShooter;
		dataSet = new StringBuilder();
		String ColumnNamesList = "encoder,imageY,CenterX,RPM";
		dataSet.append(ColumnNamesList +"\n");
		turretTarget = robotShooter.turretPosition();
	}
	
	/*Drive straight in increments of 6". Record encoder value, topRight.y, centerX, distance, RPM*/
	
	public void init(){
		Constants.navX.zeroYaw();
		robotDrive.resetPosition();
		robotDrive.enableBrake(true);
		direction = (Constants.IS_PRACTICE_BOT?1:-1);
		increment = 8;
	}
	
	public void end(){
		robotShooter.stopTimer();
		robotDrive.enableBrake(false);
	}
	
	public void driveForward(){
		if(!init){
			init = true;
			init();
		}
		if(newDrive){
			robotDrive.driveAtAngleInit(100.0, 0.0, true);
			newDrive = false;
		}
		robotPosition = robotDrive.getPosition();
		if(!inProgress){
			System.out.println(direction*((robotPosition[0]+robotPosition[1])/2) + ", " + ((increment*increment_inches)/(Constants.WHEEL_DIAM*Math.PI)));
			if( direction*((robotPosition[0]+robotPosition[1])/2) > ((increment*increment_inches)/(Constants.WHEEL_DIAM*Math.PI))){
				robotDrive.driveAtAngleUpdate(0.0,0.0, true);
				robotDrive.driveAtAngleEnd();
				inProgress = true;
			}else{
				robotDrive.driveAtAngleUpdate(100.0, 0.0, true);
			}
		}
	}
	
	public void alignTurret(){
		if(!doneBefore){
			doneBefore = true;
			do{
				robotShooter.alignTurret();
			} while(robotShooter.centeringInProgress);
		}else{
			robotShooter.getDistance();
			RPM = robotShooter.getRPM();
			turretTarget = robotShooter.turretPosition();
		}
	}
	
	public void changeRPM(double amount){
		RPM +=amount;
	}
	
	public void adjustTurret(double amount){
		turretTarget += (amount/4096);
		robotShooter.moveTurret(turretTarget - robotShooter.turretPosition());
	}
	
	public void saveFile(){
		if(inProgress){
			inProgress = false;
			try {
			    pw = new PrintWriter(new FileOutputStream(new File("/home/lvuser/calibration.csv"), true));
			    dataSet.append(data[0] + "," + data[1] + "," + data[2] + "," + data[3] + "\n");
			    SmartDashboard.putString("Calibration Data", dataSet.toString());
			    pw.write(dataSet.toString());
			    pw.close();
			    dataSet.setLength(0);
			    increment++;
			    newDrive = true;
			} catch (FileNotFoundException e) {
			    e.printStackTrace();
			}
		}
	}
	
	public void shoot(){
		robotShooter.stopTimer();
		robotShooter.Indexer(Constants.INDEXER_SPEED);	
		robotPosition = robotDrive.getPosition();
		data[0] = (robotPosition[0]+robotPosition[1])/2;
		data[1] = robotShooter.getImageTop();
		data[2] = Constants.centerX;
		data[3] = RPM;
	}
	
	public void shootStop(){
		robotShooter.StopIndexer();
    	

	}
	
	public void spinDown(){
    	robotShooter.StopShoot();
		robotShooter.StopBallPump();
		robotIntake.StopIntake();
	}
	
	public void spinUp(){
		robotShooter.Shoot(RPM);
		robotShooter.BallPump(-1);
		robotIntake.IntakeBalls(0.5);
	}

}

