package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkMax;

import frc.Constants;

public class Drive extends SubsystemBase{  
   public static SparkMax leftDriveMotor = new SparkMax(Constants.LEFT_DRIVE_MOTOR_ONE);
   public static SparkMax leftDriveSlave = new SparkMax(Constants.LEFT_DRIVE_MOTOR_TWO);
   public static SparkMax rightDriveMotor = new SparkMax(Constants.RIGHT_DRIVE_MOTOR_ONE);
   public static SparkMax rightDriveSlave = new SparkMax(Constants.RIGHT_DRIVE_MOTOR_TWO);

    public static void setupMotors(){
        leftDriveSlaveOne.follow(leftDriveMotor);
        leftDriveSlaveTwo.follow(leftDriveMotor);
        rightDriveSlaveOne.follow(rightDriveMotor);
        rightDriveSlaveTwo.follow(rightDriveMotor);

        
        leftDriveMotor.setInverted(false);
        leftDriveSlaveOne.setInverted(false);
        leftDriveSlaveTwo.setInverted(false);
        rightDriveMotor.setInverted(false);
        rightDriveSlaveOne.setInverted(false);
        rightDriveSlaveTwo.setInverted(false);


        leftDriveMotor.enableCurrentLimit(true);
        leftDriveSlaveOne.enableCurrentLimit(true);
        leftDriveSlaveTwo.enableCurrentLimit(true);
        rightDriveMotor.enableCurrentLimit(true);
        rightDriveSlaveOne.enableCurrentLimit(true);
        rightDriveSlaveTwo.enableCurrentLimit(true);

        leftDriveMotor.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT); 
        leftDriveSlaveOne.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT);
        leftDriveSlaveTwo.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT);
        rightDriveMotor.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT); 
        rightDriveSlaveOne.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT);
        rightDriveSlaveTwo.configContinuousCurrentLimit(Constants.DRIVE_TRAIN_CURRENT);

    }
    public void arcadeDrive(double speed, double turn){
        drive.arcadeDrive(speed, turn);
    }

    public static void stopMotors(){
        leftDriveMotor.stopMotor();
        leftDriveSlaveOne.stopMotor();
        leftDriveSlaveTwo.stopMotor();
        rightDriveMotor.stopMotor();
        rightDriveSlaveOne.stopMotor();
        rightDriveSlaveTwo.stopMotor();
    }


}
