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




import org.cafirst.frc.team5406.auto.DoNothing;
import org.cafirst.frc.team5406.auto.AutoStraightGear;
import org.cafirst.frc.team5406.auto.BallAuto_LeftHopperNoMP;
import org.cafirst.frc.team5406.auto.BallAuto_RightHopperNoMP;
import org.cafirst.frc.team5406.auto.AutoStraightOnly;
import org.cafirst.frc.team5406.auto.AutonomousRoutine;






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
	private Intake robotIntake = new Intake();;
	private Climber robotClimber = new Climber();;
	private Drive robotDrive = new Drive();;
	private Shooter robotShooter = new Shooter();;
	private XboxController driverGamepad;
	private XboxController operatorGamepad;
    DoubleSolenoid shiftSolenoid;
    DoubleSolenoid shooterSolenoid;
	
	private boolean gearDropHeld = true;
	private boolean driveBack = false;
	private boolean autoDone = false;
	private int autoCounter = 0;
	private AutoStraightGear straightGear;
	private AutoStraightOnly straightOnly;
	private BallAuto_LeftHopperNoMP leftBallAuto = new BallAuto_LeftHopperNoMP(robotDrive, robotIntake, robotShooter);
	private BallAuto_RightHopperNoMP rightBallAuto = new BallAuto_RightHopperNoMP(robotDrive, robotIntake, robotShooter);
    public static DigitalInput practiceBot = new DigitalInput(Constants.PRACTICE_BOT);

	private AutonomousRoutine selectedRoutine;
	private SendableChooser<Object> autonomousSelector = new SendableChooser<>();
	double turretInit = 0;
	
	



	@Override
	public void robotInit() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 180);
		camera.setFPS(30);
		Constants.IS_PRACTICE_BOT = !(practiceBot.get());
		SmartDashboard.putString("Robot Status", "Initializing");
		
		SmartDashboard.putNumber("Rand", Math.random());
		
		
		selectedRoutine = new DoNothing();
    	
    	//Initialize Drive Controller - only using one for TShirt Cannon
    	operatorGamepad = new XboxController(0);
    	driverGamepad = new XboxController(1);
    	
    	autonomousSelector.addDefault("Do Nothing", new DoNothing());
    	autonomousSelector.addObject("Drive Straight", new AutoStraightOnly(robotDrive));
    	autonomousSelector.addObject("Middle Gear", new AutoStraightGear(robotDrive, robotIntake));
    	autonomousSelector.addObject("Balls - Left Hopper", new BallAuto_LeftHopperNoMP(robotDrive, robotIntake, robotShooter));
    	autonomousSelector.addObject("Balls - Right Hopper", new BallAuto_RightHopperNoMP(robotDrive, robotIntake, robotShooter));
    	
    	SmartDashboard.putData("Autonomous", autonomousSelector);

		SmartDashboard.putString("Robot Status", "Running");

	}

	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Robot Status", "Auto");
		selectedRoutine.init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		selectedRoutine.periodic();
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
        }else{
        	robotClimber.StopClimb();
        }
		
		
		//Turn climber motors when holding Right Bumper
		/*if(driverGamepad.getRightTriggerPressed()){
			System.out.println("Climbing");
        	robotClimber.ClimbReverse();
        }else{
        	robotClimber.StopClimb();
        }*/
		
		
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
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			SmartDashboard.putBoolean("Hood Up", false);

			System.out.println("Shift");
	    	robotShooter.hoodDown(); 
        }
		
		//A-Button: Shoot Far
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			SmartDashboard.putBoolean("Hood Up", true);
			System.out.println("Hood");
	    	robotShooter.hoodUp(); 
        }
		//X-Button: Find Turret
		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			robotShooter.alignTurret();
		}
		
		//Right Trigger: spin up shooter and ball pump. Spin intake at half speed to push any stray balls into hopper 
		boolean button_pressed = false;
		if(util.applyDeadband(operatorGamepad.getRightTrigger())!=0){
			if(!button_pressed){
				robotShooter.Shoot(Math.abs(operatorGamepad.getRightTrigger())*5700);
				robotShooter.BallPump(-1);
		    	//robotIntake.IntakeBalls(0.75);
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
			robotShooter.Indexer(865);
        }else{
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
			robotShooter.turnTurret(modifier*util.applyDeadband(operatorGamepad.getRightX(), 0.2), true);
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
	
	@Override
	public void disabledInit() {
		selectedRoutine.end();
	}
	
	@Override
    public void disabledPeriodic(){  
		selectedRoutine = (AutonomousRoutine) autonomousSelector.getSelected();
		SmartDashboard.putString("Selected Autonomous", selectedRoutine.getName());
    }
}

