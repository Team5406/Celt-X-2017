package org.cafirst.frc.team5406.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.*;
import org.cafirst.frc.team5406.util.Util;
import org.cafirst.frc.team5406.util.XboxController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Servo;




import org.cafirst.frc.team5406.auto.*;







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
    public static DigitalInput practiceBot = new DigitalInput(Constants.PRACTICE_BOT);
	private Intake robotIntake = new Intake();
	private Climber robotClimber = new Climber();
	private Drive robotDrive = new Drive();
	private Shooter robotShooter = new Shooter();
	private XboxController driverGamepad;
	private XboxController operatorGamepad;
    DoubleSolenoid shiftSolenoid;
    DoubleSolenoid shooterSolenoid;
	
	private boolean gearDropHeld = true;
	private boolean driveBack = false;
	private boolean autoDone = false;
	private boolean magicTurret = false;
	private int autoCounter = 0;

	private AutonomousRoutine selectedRoutine;
	private SendableChooser<Object> autonomousSelector = new SendableChooser<>();
	double turretInit = 0;
	
	private final boolean CALIBRATION_MODE = false;
	private Calibration calibrator;
	private long teleopCounter = 0;
	private double rpm = 5800;
	private boolean setRPM = false;
	private Servo hopperFlap = new Servo(Constants.HOPPER_FLAP);



	@Override
	public void robotInit() {
		/*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 180);
		camera.setFPS(30);*/
		Constants.IS_PRACTICE_BOT = !(practiceBot.get());
		SmartDashboard.putString("Robot Status", "Initializing");
		
		SmartDashboard.putNumber("Rand", Math.random());
		
		robotShooter.cameraOffset = (Constants.IS_PRACTICE_BOT?220:272);

		selectedRoutine = new DoNothing();
    	
    	//Initialize Drive Controller - only using one for TShirt Cannon
    	operatorGamepad = new XboxController(0);
    	driverGamepad = new XboxController(1);
    	
    	autonomousSelector.addDefault("0 - Do Nothing", new DoNothing());
    	autonomousSelector.addObject("1 - Middle Gear", new AutoStraightGear(robotDrive, robotIntake));
    	autonomousSelector.addObject("2 - Balls Only - Left Hopper", new BallAuto_LeftHopper(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("3 - Balls Only - Right Hopper", new BallAuto_RightHopper(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("4 - Left Gear and Left Hopper", new GearBallLeftHopper(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("5 - Right Gear and Right Hopper", new GearBallRightHopper(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("6 - Left Gear (Left Boiler)", new AutoLeftGearLeftBoiler(robotDrive, robotIntake));
    	autonomousSelector.addObject("7 - Right Gear (Left Boiler)", new AutoRightGearLeftBoiler(robotDrive, robotIntake));
    	autonomousSelector.addObject("8 - Left Gear (Right Boiler)", new AutoLeftGearRightBoiler(robotDrive, robotIntake));
    	autonomousSelector.addObject("9 - Right Gear (Right Boiler)", new AutoRightGearRightBoiler(robotDrive, robotIntake));
    	autonomousSelector.addObject("10 - Drive Straight", new AutoStraightOnly(robotDrive));
     	autonomousSelector.addObject("11 - Middle Gear and Right Balls", new StraightGearBallRightHopper(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("12 - Middle Gear and Left Balls", new StraightGearBallLeftHopper(robotDrive, robotIntake, robotShooter));



    	
    	
    	calibrator = new Calibration(robotDrive, robotIntake, robotShooter);
    	SmartDashboard.putData("Autonomous", autonomousSelector);

		SmartDashboard.putString("Robot Status", "Running");

	}

	@Override
	public void autonomousInit() {
		Constants.auto = true;
		hopperFlap.setAngle(0);

		SmartDashboard.putString("Robot Status", "Auto");
		selectedRoutine.init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		robotDrive.getPosition();

		SmartDashboard.putNumber("Heading",Constants.navX.getYaw());

		robotShooter.getLimitSwitches();

		selectedRoutine.periodic();
	}

	@Override
	public void teleopInit() {
		Constants.auto = false;
		hopperFlap.setAngle(0);
		selectedRoutine.end();
		SmartDashboard.putString("Robot Status", "Teleop");
		robotClimber.direction_switch = false;
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robotShooter.getLimitSwitches();
		robotShooter.displayTurretPos();
		robotShooter.stoppedPID();
		robotClimber.DisplayCurrent();
		robotShooter.DisplayCurrent();
		robotDrive.getPosition();
		teleopCounter++;
		
		if(CALIBRATION_MODE){
			//STEP 1: Drive Forward
			if(util.applyDeadband(operatorGamepad.getLeftX(), 0.2)!=0){
				calibrator.driveForward();
	        }
			
			//STEP 2: Align Turret
			if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
				calibrator.alignTurret();
	        }
			
			//STEP 3: Spin Up
			if(operatorGamepad.getRightTriggerPressed()){
				calibrator.spinUp();
			}else{
				calibrator.spinDown();
			}			
			
			//STEP 4: Shoot
			if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
				calibrator.shoot();
	        }else{
				calibrator.shootStop();
	        }
			
			//STEP 5: Adjust
			switch(operatorGamepad.getDirectionPad()){
			case UP:
				calibrator.changeRPM(5);
				break;
			case DOWN:
				calibrator.changeRPM(-5);
				break;
			case LEFT:
				calibrator.adjustTurret(10);
				break;
			case RIGHT:
				calibrator.adjustTurret(-10);
				break;
			}

			//Repeat Steps 1-5
			
			//STEP 6: Save File
			if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
				calibrator.saveFile();
	        }else{
				//calibrator.shootStop();
	        }
			
			
			
			
			
		}else{
		

		/**************************
		 **** DRIVER FUNCTIONS ****
		 **************************/
		
		SmartDashboard.putNumber("Heading",Constants.navX.getYaw());
		robotShooter.DisplayCurrent();
		robotIntake.autoGearLift();


        robotDrive.ArcadeDrive(driverGamepad.getLeftX(), -1*driverGamepad.getLeftY());
		
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
		}else if(driverGamepad.getRightTriggerPressed()){
			System.out.println("Climbing Reverse");
        	robotClimber.ClimbReverse();
        }else{
        	robotClimber.StopClimb();
        }
		
		
		//Turn climber motors when holding Right Bumper
		/*if(driverGamepad.getRightTriggerPressed()){
			System.out.println("Climbing");
        	robotClimber.ClimbReverse();
        }else{
        	robotClimber.StopClimb();
        }
		
		*/
		//Turn climber motors when holding Right Bumper
		if(driverGamepad.getLeftTriggerPressed()){
			System.out.println("Precision Drive");
        	robotDrive.precisionDriveX = true;
        }else{
        	robotDrive.precisionDriveX = false;
        }
		
		if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
			System.out.println("Precision Drive");
        	robotDrive.precisionDriveY = true;
        }else{
        	robotDrive.precisionDriveY = false;
        }
		
		/**************************
		 *** OPERATOR FUNCTIONS ***
		 **************************/
		
		//Spin intake at full speed when Left Trigger pressed. Reverse intake at full speed when holding Left Bumper. Stop intake if ball pump is not enabled. 

		if(operatorGamepad.getLeftTriggerPressed()){
			SmartDashboard.putBoolean("Intaking", true);
        	robotIntake.IntakeBalls(1);
		} else if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){

		} else if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
    			System.out.println("Intake");
            	robotIntake.IntakeBalls(-1);
        }else if (util.applyDeadband(operatorGamepad.getRightX())==0){
			SmartDashboard.putBoolean("Intaking", false);
        	robotIntake.StopIntake();
        }
		

		//Y-Button: Shoot Close
		
		
		//A-Button: Find CCWLimit
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			magicTurret = false;
        }
		//X-Button: Find Turret
		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			robotShooter.alignTurret();
			//robotShooter.centerTurretIteration();
		}
		
		//Right Trigger: spin up shooter and ball pump. Spin intake at half speed to push any stray balls into hopper 
		//boolean button_pressed = false;
		if(util.applyDeadband(operatorGamepad.getRightTrigger())!=0){
			//if(!button_pressed){
				robotShooter.Shoot(rpm);
				if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
					robotShooter.BallPump(1);
				}else{
					robotShooter.BallPump(-1);					
				}
		    	//robotIntake.IntakeBalls(0.75);
		    	//button_pressed = true;
			//}
			/*if(teleopCounter < 80){
				robotShooter.BallPump(-1);
			}else if(teleopCounter < 100){
				robotShooter.BallPump(1);
			}else{
				teleopCounter=0;
				robotShooter.BallPump(-1);					
			}*/
			
    	//enable brake mode - this is passive - a PID is needed to hold the position.
		}else{
			//button_pressed = false;
	    	robotShooter.StopShoot();
			robotShooter.StopBallPump();
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			if(!operatorGamepad.getRightTriggerPressed()){
			robotShooter.BallPump(1);
			}
        }

		
		//B-Button: Shoot - run indexer to feed balls into shooter wheels.
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			SmartDashboard.putBoolean("Shooting", true);
			if(!setRPM){
				rpm = robotShooter.getRPM();
				setRPM = true;
			}
			robotShooter.Indexer(Constants.INDEXER_SPEED);
        }else{
        	setRPM = false;
			SmartDashboard.putBoolean("Shooting", false);
			robotShooter.StopIndexer();
        }
		
		
		//Right-Bumper: Drop Gear/prepare to pickup
		if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			if(!gearDropHeld){
				robotIntake.dropGear(false); //disable auto spin@Vic Park due to faulty sensor
				gearDropHeld = true;
				if(!robotIntake.haveGear()){
					robotIntake.gear_lifted = false;
				}
			}
        }else{
        	if(gearDropHeld){
        		gearDropHeld = false;
        		robotIntake.liftGear();
        	}
        }
		
		//Left Joystick: Turn Turret - with Vision
		if(util.applyDeadband(operatorGamepad.getLeftX(), 0.2)!=0){
			double modifier = 1;
			if(operatorGamepad.getButtonHeld(XboxController.LEFT_STICK)){
				modifier = 0.35;
			}
			magicTurret = false;
			robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getLeftX(), 0.2), false);
			//robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getRightX(), 0.1));
        }else{
			robotShooter.StopTurret();
        }
		
		//Right Joystick: Turn Turret - without Vision
		if(util.applyDeadband(operatorGamepad.getRightX(), 0.2)!=0){
			double modifier = 1;
			if(operatorGamepad.getButtonHeld(XboxController.LEFT_STICK)){
				modifier = 0.35;
			}
			magicTurret = false;
			robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getRightX(), 0.2), true);
			robotShooter.stopTimer();
        }else{
			robotShooter.StopTurret();
        }
		
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			robotShooter.adjustRPM(10);
			break;
		case DOWN:
			robotShooter.adjustRPM(-10);
			break;
		case LEFT:
			robotShooter.adjustTurret(10);
			robotShooter.stopTimer();
			break;
		case RIGHT:
			robotShooter.adjustTurret(-10);
			robotShooter.stopTimer();

			break;
		}
		
		
		
		}
        
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledInit() {
		Constants.auto = false;
		selectedRoutine.end();
		robotDrive.driveAtAngleEnd();
		robotShooter.stopTimer();
	}
	
	@Override
    public void disabledPeriodic(){  
		selectedRoutine = (AutonomousRoutine) autonomousSelector.getSelected();
		SmartDashboard.putString("Selected Autonomous", selectedRoutine.getName());
		robotShooter.getLimitSwitches();
    }
}

