package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Subsystems.Shooter;

import com.revrobotics.spark.SparkMax;


public class Robot extends TimedRobot {
  private SparkMax leftDriveMotor, leftDriveFollower, rightDriveMotor, rightDriveFollower;
  private XboxController driverGamepad = new XboxController(1);
  private DifferentialDrive robotDrive;
  SparkMaxConfig configBase = new SparkMaxConfig();
  SparkMaxConfig configLeft = new SparkMaxConfig();
  SparkMaxConfig configRight = new SparkMaxConfig();
  SparkMaxConfig configLeftFollower = new SparkMaxConfig();
  SparkMaxConfig configRightFollower = new SparkMaxConfig();
  

  @Override
  public void robotInit() {



    leftDriveMotor = new SparkMax(1, MotorType.kBrushless);
    leftDriveFollower = new SparkMax(2, MotorType.kBrushless);
    rightDriveMotor = new SparkMax(3, MotorType.kBrushless);
    rightDriveFollower = new SparkMax(4, MotorType.kBrushless);
   // intakeLeft = new CANSparkMax(5, MotorType.kBrushless);
  //  intakeRight = new CANSparkMax(6, MotorType.kBrushless);

  configBase
  .idleMode(SparkBaseConfig.IdleMode.kCoast)
  .smartCurrentLimit(30);

  configLeft.apply(configBase);
  configLeft.inverted(true);

  configLeftFollower.apply(configLeft);
  configLeftFollower.follow(leftDriveMotor);

  configRight.apply(configBase);
  configRightFollower.apply(configRight);
  configRightFollower.follow(rightDriveMotor);
    
    leftDriveMotor.configure(
      configLeft,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters
    );
    leftDriveFollower.configure(
      configLeftFollower,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters
    );
    rightDriveMotor.configure(
      configRight,
      SparkMax.ResetMode.kResetSafeParameters, 
      SparkMax.PersistMode.kPersistParameters
    );
    rightDriveFollower.configure(
      configRightFollower,
        SparkMax.ResetMode.kResetSafeParameters, 
        SparkMax.PersistMode.kPersistParameters
    );


   /* intakeLeft.setInverted(false);
    intakeRight.setInverted(true);
    intakeLeft.setSmartCurrentLimit(30);
    intakeRight.setSmartCurrentLimit(30); */

     robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    Shooter.setupMotors();

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void disabledInit() {

  }
  @Override
  public void teleopPeriodic() {
    robotDrive.arcadeDrive(-driverGamepad.getLeftY(), -driverGamepad.getLeftX());
    if ((Math.abs(driverGamepad.getRightX()) > 0.1)) {
      Shooter.turnTurret(driverGamepad.getRightX());
    }else{
      Shooter.turnTurret(driverGamepad.getRightX());
    }

    if ((Math.abs(driverGamepad.getRightTriggerAxis())) > 0.1) {
      Shooter.spinShooter(driverGamepad.getRightTriggerAxis());
      if (driverGamepad.getYButton()) {
        Shooter.BallPump(1);
      } else {
        Shooter.BallPump(-1);
      }
    } else {
      Shooter.stopShooter();
      Shooter.stopBallPump();
    }

    if (driverGamepad.getBButton()) {
      Shooter.Indexer(1);
    } else {
      Shooter.stopIndexer();
    }
    
  /*  if(driverGamepad.getLeftTriggerAxis() > 0.1){
      intakeLeft.set(driverGamepad.getLeftTriggerAxis());
      intakeRight.set(driverGamepad.getLeftTriggerAxis());
  
    }else if (driverGamepad.getRightTriggerAxis() > 0.1){
      intakeLeft.set(-1*driverGamepad.getRightTriggerAxis());
      intakeRight.set(-1*driverGamepad.getRightTriggerAxis());
  
    }else{
      intakeLeft.set(0);
      intakeRight.set(0);
  
    }
  } 
*/


} 

}