package org.usfirst.frc.team5406.robot;

import org.usfirst.frc.team5406.vision.GripPipeline;
import org.usfirst.frc.team5406.vision.MyThread;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	//IP of the axis camera
	final static String AXIS_IP = "10.54.6.20";
	/*IP can be found by using the Axis Camera Setup found
	 * in C:\Program Files (x86)\National Instruments\LabVIEW 2016\project\Axis Camera Tool
	 * It may take time to find the IP but give it a few minutes*/
	
	final static int TURRET_MOTOR = 9; //Not sure what actual ID is but i like the number 9
	
	//Camera Specs
	final static double IMAGE_WIDTH = 320;
	final static double IMAGE_HEIGHT = 240;
	final static double IMAGE_CENTER = IMAGE_WIDTH / 2;
	
	
	//Center of the contour
	public static double centerX = 0.0;
	
	CANTalon turretMotor;
	
	//Filters Image, You must GripPipeline.java into the new Robot code for it to work
	GripPipeline gripPipeline;
	//Camera being used
	AxisCamera axisCamera;
	//Runs Vision Scanning
	MyThread thread;
	
	
	@Override
	public void robotInit() {
		
		System.out.println("INIT");
		
		turretMotor = new CANTalon(TURRET_MOTOR);
		
		//For when the turret is needed
		turretMotor.changeControlMode(TalonControlMode.PercentVbus);
		turretMotor.setVoltageRampRate(50);
		
		turretMotor.configPeakOutputVoltage(12, -12);
		turretMotor.configNominalOutputVoltage(0, 0);
		
		//Instantiate items
		gripPipeline = new GripPipeline();
		axisCamera = CameraServer.getInstance().addAxisCamera(AXIS_IP);
		thread = new MyThread(gripPipeline, axisCamera);
		
		//Starts and runs thread
		thread.start();
		
		//Centering code is found in teleop
		
		
        System.out.println("INIT Done");
        
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//Displays center of contour on dashboard
		SmartDashboard.putNumber("Contour", centerX);
		
		//Power of the Motor
		double speed = 0;
		
		//How much error the camera can be at
		double tolerance = 2;
		
		//Chooses the right power for the motor
		if(centerX == 0)
			speed = 0;
		else if(centerX > IMAGE_CENTER + tolerance)
			speed = 0.07 + 0.3 * ((centerX - IMAGE_CENTER) / IMAGE_CENTER);
		else if(centerX < IMAGE_CENTER - tolerance)
			speed = -0.07 - 0.3 * ((IMAGE_CENTER - centerX) / IMAGE_CENTER);
		
		//Powers motor
		turretMotor.set(speed);
		SmartDashboard.putNumber("Speed", speed);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

