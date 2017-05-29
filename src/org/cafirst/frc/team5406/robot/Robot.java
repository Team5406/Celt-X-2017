package org.cafirst.frc.team5406.robot;

import org.cafirst.frc.team5406.auto.AutoLeftGearLeftBoiler;
import org.cafirst.frc.team5406.auto.AutoLeftGearRightBoiler;
import org.cafirst.frc.team5406.auto.AutoMiddleGear;
import org.cafirst.frc.team5406.auto.AutoRightGearLeftBoiler;
import org.cafirst.frc.team5406.auto.AutoRightGearRightBoiler;
import org.cafirst.frc.team5406.auto.AutoStraightOnly;
import org.cafirst.frc.team5406.auto.AutonomousRoutine;
import org.cafirst.frc.team5406.auto.BallAutoLeftHopper;
import org.cafirst.frc.team5406.auto.BallAutoMiddleGearLeftBoiler;
import org.cafirst.frc.team5406.auto.BallAutoMiddleGearRightBoiler;
import org.cafirst.frc.team5406.auto.BallAutoRightHopper;
import org.cafirst.frc.team5406.auto.DoNothing;
import org.cafirst.frc.team5406.controller.ControllerBase;
import org.cafirst.frc.team5406.controller.XboxController;
import org.cafirst.frc.team5406.controller.XboxController.XboxButton;
import org.cafirst.frc.team5406.subsystem.Climber;
import org.cafirst.frc.team5406.subsystem.Drive;
import org.cafirst.frc.team5406.subsystem.Intake;
import org.cafirst.frc.team5406.subsystem.Shooter;
import org.cafirst.frc.team5406.util.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	private Drive robotDrive;
	private Intake robotIntake;
	private Shooter robotShooter;
	private Climber robotClimber;
	
	private DigitalInput practiceBotIndicator;
	
	private XboxController driverController;
	private XboxController operatorController;
	
	private SendableChooser<AutonomousRoutine> autoChooser;
	private AutonomousRoutine selectedRoutine;
	private AutonomousRoutine actualRoutine;
	
	private double shooterRPM = 5800;
	private boolean isRPMSet = false;
	
	private boolean isGearDropHeld = false;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		SmartDashboard.putString("Robot Status", "Initializing");
		
		robotDrive = new Drive();
		robotIntake = new Intake();
		robotShooter = new Shooter();
		robotClimber = new Climber();
		
		practiceBotIndicator = new DigitalInput(Constants.PRACTICE_BOT_INDICATOR);
		Constants.IS_PRACTICE_BOT = !(practiceBotIndicator.get());
		
		operatorController = new XboxController(0);
		driverController = new XboxController(1);
		
		autoChooser = new SendableChooser<AutonomousRoutine>();
		selectedRoutine = new DoNothing();
		actualRoutine = selectedRoutine.newInstance();
		
		robotShooter.setCameraOffset((Constants.IS_PRACTICE_BOT ? 220 : 270));
		
    	autoChooser.addDefault("0 - Do Nothing", new DoNothing());
    	autoChooser.addObject("1 - Middle Gear", new AutoMiddleGear(robotDrive, robotIntake));
    	autoChooser.addObject("2 - Balls Only - Left Hopper", new BallAutoLeftHopper(robotDrive, robotShooter, robotIntake));
    	autoChooser.addObject("3 - Balls Only - Right Hopper", new BallAutoRightHopper(robotDrive, robotShooter, robotIntake));
    	autoChooser.addObject("4 - Left Gear and Left Hopper", new BallAutoLeftHopper(robotDrive, robotShooter, robotIntake));
    	autoChooser.addObject("5 - Right Gear and Right Hopper", new BallAutoRightHopper(robotDrive, robotShooter, robotIntake));
    	autoChooser.addObject("6 - Left Gear (Left Boiler)", new AutoLeftGearLeftBoiler(robotDrive, robotIntake));
    	autoChooser.addObject("7 - Right Gear (Left Boiler)", new AutoRightGearLeftBoiler(robotDrive, robotIntake));
    	autoChooser.addObject("8 - Left Gear (Right Boiler)", new AutoLeftGearRightBoiler(robotDrive, robotIntake));
    	autoChooser.addObject("9 - Right Gear (Right Boiler)", new AutoRightGearRightBoiler(robotDrive, robotIntake));
    	autoChooser.addObject("10 - Drive Straight", new AutoStraightOnly(robotDrive));
     	autoChooser.addObject("11 - Middle Gear and Right Balls", new BallAutoMiddleGearRightBoiler(robotDrive, robotShooter, robotIntake));
    	autoChooser.addObject("12 - Middle Gear and Left Balls", new BallAutoMiddleGearLeftBoiler(robotDrive, robotShooter, robotIntake));
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
		actualRoutine = selectedRoutine.newInstance();
		actualRoutine.init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		robotDrive.getPosition();
		
		robotShooter.displayLimitSwitches();

		actualRoutine.periodic();
	}
	
	@Override
	public void teleopInit()
	{
		actualRoutine.end();
		robotClimber.setDirectionSwitched(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robotShooter.displayLimitSwitches();
		robotDrive.getPosition();
		
		//Driver Controls
		robotDrive.arcadeDrive(driverController.getLeftX(), -1 * driverController.getLeftY());
		
		//Default to low gear, shift high when holding B
		if(driverController.getButtonHeld(XboxButton.B_BUTTON))
	    	robotDrive.shiftHigh();
        else
        	robotDrive.shiftLow();
		
		//Turn climber motors when holding Right Bumper
		if(driverController.getButtonHeld(XboxButton.RIGHT_BUMPER))
        	robotClimber.climb();
		else if(driverController.getRightTriggerPressed())
        	robotClimber.climbReverse();
        else
        	robotClimber.climbStop();
		
		//Turn climber motors when holding Right Bumper
		if(driverController.getLeftTriggerPressed())
        	robotDrive.setPrecisionDriveX(true);
        else
        	robotDrive.setPrecisionDriveX(false);
		
		if(driverController.getButtonHeld(XboxButton.LEFT_BUMPER))
        	robotDrive.setPrecisionDriveY(true);
        else
        	robotDrive.setPrecisionDriveY(false);
        
		/**************************
		 *** OPERATOR FUNCTIONS ***
		 **************************/
		
		//Spin intake at full speed when Left Trigger pressed. Reverse intake at full speed when holding Left Bumper. Stop intake if ball pump is not enabled. 
		if(operatorController.getLeftTriggerPressed())
        	robotIntake.intakeBalls(1);
		else if(operatorController.getButtonHeld(XboxButton.LEFT_BUMPER))
            	robotIntake.intakeBalls(-1);
        else if(ControllerBase.applyDeadband(operatorController.getRightX()) == 0)
        	robotIntake.stopIntake();
        
		//X-Button: Find Turret
		if(operatorController.getButtonHeld(XboxButton.X_BUTTON))
			robotShooter.alignTurret();
		
		//Right Trigger: spin up shooter and ball pump. Spin intake at half speed to push any stray balls into hopper 
		if(ControllerBase.applyDeadband(operatorController.getRightTrigger()) != 0){
			robotShooter.shoot(shooterRPM);
			if(operatorController.getButtonHeld(XboxButton.Y_BUTTON))
				robotShooter.setBallPump(1);
			else
				robotShooter.setBallPump(-1);						
		}else{
	    	robotShooter.stopShoot();
			robotShooter.stopBallPump();
		}
		
		if(operatorController.getButtonHeld(XboxButton.Y_BUTTON))
			if(!operatorController.getRightTriggerPressed())
				robotShooter.setBallPump(1);
		
		//B-Button: Shoot - run indexer to feed balls into shooter wheels.
		if(operatorController.getButtonHeld(XboxButton.B_BUTTON)){
			if(!isRPMSet)
			{
				shooterRPM = robotShooter.getRPM();
				isRPMSet = true;
			}
			robotShooter.setIndexer(Constants.indexer.getTarget());
        }else{
        	isRPMSet = false;
			robotShooter.stopIndexer();
        }
		
		//Right-Bumper: Drop Gear/prepare to pickup
		if(operatorController.getButtonHeld(XboxButton.RIGHT_BUMPER)){
			if(!isGearDropHeld)
			{
				robotIntake.dropGear(false);
				isGearDropHeld = true;
			}
        }else{
        	if(isGearDropHeld){
        		isGearDropHeld = false;
        		robotIntake.liftGear();
        	}
        }
		
		//Left Joystick: Turn Turret - with Vision
		if(ControllerBase.applyDeadband(operatorController.getLeftX(), 0.2) != 0)
		{
			double modifier = 1;
			if(operatorController.getButtonHeld(XboxButton.LEFT_STICK))
				modifier = 0.35;
			
			robotShooter.turnTurret(modifier * ControllerBase.applyDeadband(operatorController.getLeftX(), 0.2), false);
        }
		
		//Right Joystick: Turn Turret - without Vision
		if(ControllerBase.applyDeadband(operatorController.getRightX(), 0.2) != 0)
		{
			double modifier = 1;
			if(operatorController.getButtonHeld(XboxButton.LEFT_STICK)){
				modifier = 0.35;
			}
			
			robotShooter.turnTurret(modifier * ControllerBase.applyDeadband(operatorController.getRightX(), 0.2), true);
			robotShooter.stopTimer();
        }
		
		switch(operatorController.getDirectionPad())
		{
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
		default:
			break;
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledInit() 
	{
		actualRoutine.end();
		robotDrive.driveAtAngleEnd();
		robotShooter.stopTimer();
	}
	
	@Override
    public void disabledPeriodic()
	{  
		selectedRoutine = autoChooser.getSelected();
		SmartDashboard.putString("Selected Autonomous", selectedRoutine.getName());
		SmartDashboard.putString("Actual Autonomous", actualRoutine.getName());
		robotShooter.displayLimitSwitches();
    }
}

