package org.cafirst.frc.team5406.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.*;
import org.cafirst.frc.team5406.util.Util;
import org.cafirst.frc.team5406.util.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	Constants constants = new Constants();
	Util util = new Util();
	private Intake robotIntake;
	private Climber robotClimber;
	private Drive robotDrive;
	private Shooter robotShooter;
	private XboxController driverGamepad;
	private XboxController operatorGamepad;
    DoubleSolenoid shiftSolenoid;
    DoubleSolenoid shooterSolenoid;
	//public AHRS navX = new AHRS(SPI.Port.kMXP);
	



	@Override
	public void robotInit() {
		SmartDashboard.putString("Robot Status", "Initializing");
		robotIntake = new Intake();
		robotClimber = new Climber();
		robotDrive = new Drive();
		robotShooter = new Shooter();
		
		SmartDashboard.putNumber("Rand", Math.random());
		

    	
    	//Initialize Drive Controller - only using one for TShirt Cannon
    	operatorGamepad = new XboxController(0);
    	driverGamepad = new XboxController(1);
    	
		SmartDashboard.putString("Robot Status", "Running");

	}

	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Robot Status", "Auto");

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		SmartDashboard.putString("Robot Status", "Teleop");
		robotClimber.direction_switch = false;
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		/**************************
		 **** DRIVER FUNCTIONS ****
		 **************************/
		
		//SmartDashboard.putNumber("Heading",navX.getPitch());
		/*robotIntake.DisplayCurrent();
		robotClimber.DisplayCurrent();
		robotDrive.DisplayCurrent();*/
		robotShooter.DisplayCurrent();


        robotDrive.ArcadeDrive(-1*driverGamepad.getLeftY(), driverGamepad.getLeftX());
		
		//Default to low gear, shift high when holding B
		if(driverGamepad.getButtonHeld(XboxController.B_BUTTON)){
			System.out.println("Shift");
	    	robotDrive.shiftHigh();
        }else{
        	robotDrive.shiftLow();
        }
		
		//Turn climber motors when holding Right Bumper
		if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			System.out.println("Climbing");
        	robotClimber.Climb();
        }else{
        	robotClimber.StopClimb();
        }
		
		//Turn climber motors when holding Right Bumper
		if(driverGamepad.getLeftTriggerPressed()){
			System.out.println("Precision Drive");
        	robotDrive.precisionDrive = true;
        }else{
        	robotDrive.precisionDrive = false;
        }
		
		/**************************
		 *** OPERATOR FUNCTIONS ***
		 **************************/
		//Spin intake at full speed when Left Trigger pressed. Reverse intake at full speed when holding Left Bumper. Stop intake if ball pump is not enabled. 
		if(operatorGamepad.getLeftTriggerPressed()){
			SmartDashboard.putBoolean("Intaking", true);
        	robotIntake.IntakeBalls(-1);
		}else if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
    			System.out.println("Intake");
            	robotIntake.IntakeBalls(1);
        }else if (util.applyDeadband(operatorGamepad.getRightX())==0){
			SmartDashboard.putBoolean("Intaking", false);
        	robotIntake.StopIntake();
        }
		
		robotShooter.displayEnc();

		//Y-Button: Shoot Close
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			SmartDashboard.putBoolean("Hood Up", false);

			System.out.println("Shift");
	    	robotShooter.hoodDown(); 
        }
		
		//Y-Button: Shoot Far
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			SmartDashboard.putBoolean("Hood Up", true);
			System.out.println("Hood");
	    	robotShooter.hoodUp(); 
        }
		
		//Right Trigger: spin up shooter and ball pump. Spin intake at half speed to push any stray balls into hopper 
		boolean button_pressed = false;
		if(util.applyDeadband(operatorGamepad.getRightTrigger())!=0){
			if(!button_pressed){
				robotShooter.Shoot(Math.abs(operatorGamepad.getRightTrigger())*4500);
				robotShooter.BallPump(-1);
		    	robotIntake.IntakeBalls(-0.5);
		    	button_pressed = true;
			}
    	//enable brake mode - this is passive - a PID is needed to hold the position.
		}else{
			button_pressed = false;
	    	robotShooter.StopShoot();
			robotShooter.StopBallPump();
		}
		

		
		//B-Button: Shoot - run indexer to feed balls into shooter wheels.
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			SmartDashboard.putBoolean("Shooting", true);
			robotShooter.Indexer(-1);
        }else{
			SmartDashboard.putBoolean("Shooting", false);
			robotShooter.StopIndexer();
        }
		
		
		if(util.applyDeadband(operatorGamepad.getRightX(), 0.2)!=0){
			double modifier = 0.5;
			if(operatorGamepad.getButtonHeld(XboxController.RIGHT_STICK)){
				modifier = 0.35;
			}
			robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getRightX(), 0.2));
			//robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getRightX(), 0.1));
        }else{
			robotShooter.StopTurret();
        }
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

