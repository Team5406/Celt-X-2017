package org.cafirst.frc.team5406.util;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;

public class Constants {
	
	//Drive Motors
	public static final int LEFT_DRIVE_MASTER_ID = 1;
	public static final int[] LEFT_DRIVE_FOLLOWER_IDS = {2, 3};
	public static final int RIGHT_DRIVE_MASTER_ID = 4;
	public static final int[] RIGHT_DRIVE_FOLLOWER_IDS = {5, 6};
	public static final int DRIVE_ENCODER_RATIO = 1;
	public static final int DRIVE_STALL_CURRENT = 131;
	public static final double DRIVE_FREE_CURRENT = 2.7;
	public static final int DRIVE_FREE_SPEED = 5330;
	public static final int DRIVE_GEARING = 9;
	public static final int DRIVE_TARGET = 600;
	public static final TalonControlMode DRIVE_MODE = CANTalon.TalonControlMode.PercentVbus;
	public static final boolean DRIVE_INVERT = true;
	public static final boolean DRIVE_REVERSE_ENCODER = true;
	public static final double DRIVE_PID_F = 0.3;
	public static final double DRIVE_PID_P=0.008; 
	public static final double DRIVE_PID_I=0.00000; 
	public static final double DRIVE_PID_D=0.0003;
	
	//Intake Motor
	public static final int INTAKE_ID = 7;
	public static final int INTAKE_ENCODER_RATIO = 1;
	public static final int INTAKE_STALL_CURRENT = 89;
	public static final double INTAKE_FREE_CURRENT = 3;
	public static final int INTAKE_FREE_SPEED =5840;
	public static final int INTAKE_GEARING = 4;
	public static final int INTAKE_TARGET =1200;
	public static final boolean INTAKE_REVERSE_ENCODER = true; 
	public static final TalonControlMode INTAKE_MODE = CANTalon.TalonControlMode.PercentVbus;
	public static final boolean INTAKE_INVERT = false;
	public static final double INTAKE_PID_F = (1023 * 600) / (INTAKE_TARGET * 4096);
	public static final double INTAKE_PID_P = 0.05 * 1023;
	public static final double INTAKE_PID_I=0.00; 
	public static final double INTAKE_PID_D = 10 * INTAKE_PID_P;
	
	//Ball Pump Motor
	public static final int BALL_PUMP_ID = 8;
	public static final int BALL_PUMP_ENCODER_RATIO = 1;
	public static final int BALL_PUMP_STALL_CURRENT = 89;
	public static final double BALL_PUMP_FREE_CURRENT = 3;
	public static final int BALL_PUMP_FREE_SPEED = 5840;
	public static final int BALL_PUMP_GEARING = 7;
	public static final int BALL_PUMP_TARGET = 500;
	public static final TalonControlMode BALL_PUMP_MODE = CANTalon.TalonControlMode.PercentVbus;
	public static final boolean BALL_PUMP_REVERSE_ENCODER = false;
	public static final boolean BALL_PUMP_INVERT = false;
	public static final double BALL_PUMP_PID_F = (1023 * 600) / (BALL_PUMP_TARGET * 4096);
	public static final double BALL_PUMP_PID_P = 0.05 * 1023;
	public static final double BALL_PUMP_PID_I = 0.00;
	public static final double BALL_PUMP_PID_D = 10 * BALL_PUMP_PID_P;

	//Indexer Motor
	public static final int INDEXER_ID = 9;
	public static final int INDEXER_ENCODER_RATIO = 1;
	public static final int INDEXER_STALL_CURRENT = 41;
	public static final double INDEXER_FREE_CURRENT = 1.8;
	public static final int INDEXER_FREE_SPEED = 13180;
	public static final int INDEXER_GEARING = 10;
	public static final int INDEXER_TARGET = 900;
	public static final TalonControlMode INDEXER_MODE = CANTalon.TalonControlMode.PercentVbus;
	public static final boolean INDEXER_REVERSE_ENCODER = false;
	public static final boolean INDEXER_INVERT = false;
	public static final double INDEXER_PID_F = -1 * (1023 * 600) / (INDEXER_TARGET * 4096);
	public static final double INDEXER_PID_P = 0.03;
	public static final double INDEXER_PID_I = 0.00;
	public static final double INDEXER_PID_D = 10 * INDEXER_PID_P;
	
	//Shooter Motors
	public static final int SHOOTER_MASTER_ID = 10;
	public static final int SHOOTER_SLAVE_ID = 11;
	public static final int SHOOTER_ENCODER_RATIO = 1;
	public static final int SHOOTER_STALL_CURRENT = 134;
	public static final double SHOOTER_FREE_CURRENT = 0.7;
	public static final int SHOOTER_FREE_SPEED = 18730; //free_speed
	public static final int SHOOTER_GEARING = 1; //gearing
	public static final int SHOOTER_TARGET = 5700; //rpm
	public static final TalonControlMode SHOOTER_MODE = CANTalon.TalonControlMode.Speed;
	public static final boolean SHOOTER_REVERSE_ENCODER = false; //reverse_encoder
	public static final boolean SHOOTER_INVERT = false;
	public static final double SHOOTER_PID_F = 0.0132; //PID_F
	public static final double SHOOTER_PID_P = 0.04; //PID_P //PracticeBOT: 0.06, CompBot: 0.04
	public static final double SHOOTER_PID_I = 0.0000; //PID_I
	public static final double SHOOTER_PID_D = 0.006; //PID_D
	
	//Turret Motor
	public static final int TURRET_ID = 12;
	public static final int TURRET_ENCODER_RATIO = 1;
	public static final int TURRET_STALL_CURRENT = 41;
	public static final double TURRET_FREE_CURRENT = 1.8;
	public static final int TURRET_FREE_SPEED = 13180;
	public static final int TURRET_GEARING = 10;
	public static final int TURRET_TARGET = 100;
	public static final TalonControlMode TURRET_MODE = CANTalon.TalonControlMode.MotionMagic;
	public static final boolean TURRET_REVERSE_ENCODER = false;
	public static final boolean TURRET_INVERT = false;
	public static final double TURRET_PID_F = 0.3;
	public static final double TURRET_PID_P = 6;
	public static final double TURRET_PID_I = 0.0;
	public static final double TURRET_PID_D = 5;
	
	//Climber Motors
	public static final int CLIMBER_MASTER_ID = 13;
	public static final int CLIMBER_SLAVE_ID = 14;
	public static final int CLIMBER_ENCODER_RATIO = 1;
	public static final int CLIMBER_STALL_CURRENT = 41;
	public static final double CLIMBER_FREE_CURRENT = 1.8;
	public static final int CLIMBER_FREE_SPEED = 13180; //free_speed
	public static final int CLIMBER_GEARING = 45; //gearing
	public static final int CLIMBER_TARGET = 100; //rpm
	public static final TalonControlMode CLIMBER_MODE = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
	public static final boolean CLIMBER_REVERSE_ENCODER = false; //reverse_encoder
	public static final boolean CLIMBER_INVERT = false;
	public static final double CLIMBER_PID_F = (1023 * 600) / (CLIMBER_TARGET * 4096); //PID_F
	public static final double CLIMBER_PID_P = 0.05 * 1023; //PID_P
	public static final double CLIMBER_PID_I = 0.00; //PID_I
	public static final double CLIMBER_PID_D = 10 * CLIMBER_PID_P; //PID_D
	
	//Gearboxes
	public static Gearbox drive = new Gearbox();
	public static Gearbox intake = new Gearbox();
	public static Gearbox ballPump = new Gearbox();
	public static Gearbox indexer = new Gearbox();
	public static Gearbox shooter = new Gearbox();
	public static Gearbox turret = new Gearbox();
	public static Gearbox climber = new Gearbox();
	
	//Sets Values for Gearboxes
	static
	{
		//Drive
		drive.setEncoderRatio(DRIVE_ENCODER_RATIO);
		drive.setStallCurrent(DRIVE_STALL_CURRENT);
		drive.setFreeCurrent(DRIVE_FREE_CURRENT);
		drive.setFreeSpeed(DRIVE_FREE_SPEED);
		drive.setGearing(DRIVE_GEARING);
		drive.setTarget(DRIVE_TARGET);
		drive.setMode(DRIVE_MODE);
		drive.setInvert(DRIVE_INVERT);
		drive.setReverseEncoder(DRIVE_REVERSE_ENCODER);
		drive.setPID_F(DRIVE_PID_F);
		drive.setPID_P(DRIVE_PID_P);
		drive.setPID_I(DRIVE_PID_I);
		drive.setPID_D(DRIVE_PID_D);
		
		//Intake
		intake.setEncoderRatio(INTAKE_ENCODER_RATIO);
		intake.setStallCurrent(INTAKE_STALL_CURRENT);
		intake.setFreeCurrent(INTAKE_FREE_CURRENT);
		intake.setFreeSpeed(INTAKE_FREE_SPEED);
		intake.setGearing(INTAKE_GEARING);
		intake.setTarget(INTAKE_TARGET);
		intake.setReverseEncoder(INTAKE_REVERSE_ENCODER);
		intake.setMode(INTAKE_MODE);
		intake.setInvert(INTAKE_INVERT);
		intake.setPID_F(INTAKE_PID_F);
		intake.setPID_P(INTAKE_PID_P);
		intake.setPID_I(INTAKE_PID_I);
		intake.setPID_D(INTAKE_PID_D);
		
		//BallPump
		ballPump.setEncoderRatio(BALL_PUMP_ENCODER_RATIO);
		ballPump.setStallCurrent(BALL_PUMP_STALL_CURRENT);
		ballPump.setFreeCurrent(BALL_PUMP_FREE_CURRENT);
		ballPump.setFreeSpeed(BALL_PUMP_FREE_SPEED);
		ballPump.setGearing(BALL_PUMP_GEARING);
		ballPump.setTarget(BALL_PUMP_TARGET);
		ballPump.setMode(BALL_PUMP_MODE);
		ballPump.setReverseEncoder(BALL_PUMP_REVERSE_ENCODER);
		ballPump.setInvert(BALL_PUMP_INVERT);
		ballPump.setPID_F(BALL_PUMP_PID_F);
		ballPump.setPID_P(BALL_PUMP_PID_P);
		ballPump.setPID_I(BALL_PUMP_PID_I);
		ballPump.setPID_D(BALL_PUMP_PID_D);
		
		//Indexer
		indexer.setEncoderRatio(INDEXER_ENCODER_RATIO);
		indexer.setStallCurrent(INDEXER_STALL_CURRENT);
		indexer.setFreeCurrent(INDEXER_FREE_CURRENT);
		indexer.setFreeSpeed(INDEXER_FREE_SPEED);
		indexer.setGearing(INDEXER_GEARING);
		indexer.setTarget(INDEXER_TARGET);
		indexer.setMode(INDEXER_MODE);
		indexer.setReverseEncoder(INDEXER_REVERSE_ENCODER);
		indexer.setInvert(INDEXER_INVERT);
		indexer.setPID_F(INDEXER_PID_F);
		indexer.setPID_P(INDEXER_PID_P);
		indexer.setPID_I(INDEXER_PID_I);
		indexer.setPID_D(INDEXER_PID_D);

		//Shooter
		shooter.setEncoderRatio(SHOOTER_ENCODER_RATIO);
		shooter.setStallCurrent(SHOOTER_STALL_CURRENT);
		shooter.setFreeCurrent(SHOOTER_FREE_CURRENT);
		shooter.setFreeSpeed(SHOOTER_FREE_SPEED);
		shooter.setGearing(SHOOTER_GEARING);
		shooter.setTarget(SHOOTER_TARGET);
		shooter.setMode(SHOOTER_MODE);
		shooter.setReverseEncoder(SHOOTER_REVERSE_ENCODER);
		shooter.setInvert(SHOOTER_INVERT);
		shooter.setPID_F(SHOOTER_PID_F);
		shooter.setPID_P(SHOOTER_PID_P);
		shooter.setPID_I(SHOOTER_PID_I);
		shooter.setPID_D(SHOOTER_PID_D);
		
		//Turret
		turret.setEncoderRatio(TURRET_ENCODER_RATIO);
		turret.setStallCurrent(TURRET_STALL_CURRENT);
		turret.setFreeCurrent(TURRET_FREE_SPEED);
		turret.setFreeSpeed(TURRET_FREE_SPEED);
		turret.setGearing(TURRET_GEARING);
		turret.setTarget(TURRET_TARGET);
		turret.setMode(TURRET_MODE);
		turret.setReverseEncoder(TURRET_REVERSE_ENCODER);
		turret.setInvert(TURRET_INVERT);
		turret.setPID_F(TURRET_PID_F);
		turret.setPID_P(TURRET_PID_P);
		turret.setPID_I(TURRET_PID_I);
		turret.setPID_D(TURRET_PID_D);
		
		//Climber
		climber.setEncoderRatio(TURRET_ENCODER_RATIO);
		climber.setStallCurrent(CLIMBER_STALL_CURRENT);
		climber.setFreeCurrent(CLIMBER_FREE_CURRENT);
		climber.setFreeSpeed(CLIMBER_FREE_SPEED);
		climber.setGearing(CLIMBER_GEARING);
		climber.setTarget(CLIMBER_TARGET);
		climber.setMode(CLIMBER_MODE);
		climber.setReverseEncoder(CLIMBER_REVERSE_ENCODER);
		climber.setInvert(CLIMBER_INVERT);
		climber.setPID_F(CLIMBER_PID_F);
		climber.setPID_P(CLIMBER_PID_P);
		climber.setPID_I(CLIMBER_PID_I);
		climber.setPID_D(CLIMBER_PID_D);
	}
	
	public static boolean IS_PRACTICE_BOT = false;
	
	public static final double CONTROLLER_DEADBAND = 0.2;
	
	//DIO port
	public static final int CCW_LIMIT_SWITCH = 0;
	public static final int CW_LIMIT_SWITCH = 1;
	public static final int GEAR_SWITCH = 2;
	public static final int GEAR_LIGHT = 4;
	public static final int PRACTICE_BOT_INDICATOR = 9;
	
	//PCM ports
	public static final int GEAR_GRIP_REVERSE = 4;
	public static final int GEAR_GRIP_FORWARD = 5;
	public static final int GEAR_LIFT_FORWARD = 6;
	public static final int GEAR_LIFT_REVERSE = 7;
	
	public static final DoubleSolenoid.Value GEAR_LIFT_DOWN = DoubleSolenoid.Value.kForward;
	public static final DoubleSolenoid.Value GEAR_LIFT_UP = DoubleSolenoid.Value.kReverse;
	public static final DoubleSolenoid.Value GEAR_GRIP_CLOSED = DoubleSolenoid.Value.kForward;
	public static final DoubleSolenoid.Value GEAR_GRIP_OPEN = DoubleSolenoid.Value.kReverse;
	
	public static int FOR_LIMIT_POSITION = Integer.MAX_VALUE;
	public static int REV_LIMIT_POSITION = Integer.MIN_VALUE;
	
	public static final int SHIFT_FORWARD = 0;
	public static final int SHIFT_REVERSE = 1;
	public static final DoubleSolenoid.Value SHIFT_HIGH = DoubleSolenoid.Value.kForward;
	public static final DoubleSolenoid.Value SHIFT_LOW = DoubleSolenoid.Value.kReverse;
	
	public static final AHRS navX = new AHRS(SPI.Port.kMXP);
	public static final double GYRO_PID_P = 0.025;
	public static final double GYRO_PID_I = 0.000;
	public static final double GYRO_PID_D = 0.125;
	
	public static final String AXIS_IP = "10.54.6.17";
	
	public static final int IMAGE_WIDTH = 480;
	
	public static final int AXIS_FOV = 63;
	public static final int TURRET_ROTATION_DEG = 309;
	public static final int TURRET_ROTATION_TICKS = 29580;
	
	public static final int WHEEL_DIAM = 4;
	public static final int WHEEL_TRACK = 34;
	
	public static final int ROBOT_LENGTH = 36;
}
