package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.Constants;

public class Shooter extends SubsystemBase {
    public static WPI_TalonSRX shooterMaster = new WPI_TalonSRX(Constants.SHOOTER_WHEEL_MOTOR_ONE);
    public static WPI_TalonSRX shooterSlave = new WPI_TalonSRX(Constants.SHOOTER_WHEEL_MOTOR_TWO);

    public static WPI_TalonSRX turret = new WPI_TalonSRX(Constants.TURRET_MOTOR);

    public static WPI_TalonSRX ballPump = new WPI_TalonSRX(Constants.BALL_PUMP_MOTOR);
    
    public static WPI_TalonSRX indexer = new WPI_TalonSRX(Constants.INDEXER_MOTOR);

    public static void setupMotors(){
        shooterSlave.follow(shooterMaster);

        shooterMaster.enableCurrentLimit(true);
        shooterSlave.enableCurrentLimit(true);
        turret.enableCurrentLimit(true);
        ballPump.enableCurrentLimit(true);
        indexer.enableCurrentLimit(true);

        shooterMaster.configContinuousCurrentLimit(Constants.SHOOTER_CURRENT);
        shooterSlave.configContinuousCurrentLimit(Constants.SHOOTER_CURRENT);
        turret.configContinuousCurrentLimit(Constants.TURRET_CURRENT);
        ballPump.configContinuousCurrentLimit(Constants.BALL_PUMP_CURRENT);
        indexer.configContinuousCurrentLimit(Constants.INDEXER_CURRENT);
        indexer.setInverted(true);
	    shooterMaster.configNominalOutputForward(0, Constants.KTIMEOUTMS);
	    shooterMaster.configNominalOutputReverse(0, Constants.KTIMEOUTMS);
	    shooterMaster.configPeakOutputForward(1, Constants.KTIMEOUTMS);
	    shooterMaster.configPeakOutputReverse(-1, Constants.KTIMEOUTMS);


        shooterMaster.config_kF(Constants.KSLOTIDX, Constants.SHOOTER_PID_F);
        shooterMaster.config_kP(Constants.KSLOTIDX, Constants.SHOOTER_PID_P);
        shooterMaster.config_kI(Constants.KSLOTIDX, Constants.SHOOTER_PID_I);
        shooterMaster.config_kD(Constants.KSLOTIDX, Constants.SHOOTER_PID_D);

        shooterSlave.config_kF(Constants.KSLOTIDX, Constants.SHOOTER_PID_F);
        shooterSlave.config_kP(Constants.KSLOTIDX, Constants.SHOOTER_PID_P);
        shooterSlave.config_kI(Constants.KSLOTIDX, Constants.SHOOTER_PID_I);
        shooterSlave.config_kD(Constants.KSLOTIDX, Constants.SHOOTER_PID_D);

        shooterMaster.setInverted(true);
        shooterSlave.setInverted(false);
        turret.config_kF(Constants.KSLOTIDX, Constants.TURRET_PID_F);
        turret.config_kP(Constants.KSLOTIDX, Constants.TURRET_PID_P);
        turret.config_kI(Constants.KSLOTIDX, Constants.TURRET_PID_I);
        turret.config_kD(Constants.KSLOTIDX, Constants.TURRET_PID_D);


        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.KTIMEOUTMS);
		turret.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.KTIMEOUTMS);

		/* Set the peak and nominal outputs */
		turret.configNominalOutputForward(0, Constants.KTIMEOUTMS);
		turret.configNominalOutputReverse(0, Constants.KTIMEOUTMS);
		turret.configPeakOutputForward(1, Constants.KTIMEOUTMS);
		turret.configPeakOutputReverse(-1, Constants.KTIMEOUTMS);

		/* Set Motion Magic gains in slot0 - see documentation */
        
        turret.configMotionCruiseVelocity(2000, Constants.KTIMEOUTMS);
        turret.configMotionAcceleration(2000, Constants.KTIMEOUTMS);
        turret.setInverted(true);

    }
    public static void spinShooter(double value){

        if(value == 0){
            stopShooter();
        } else {

            //double targetVelocity_UnitsPer100ms = Constants.SHOOTER_SCALING_FACTOR*(Constants.NUMBER_OF_BUCKETS*Constants.ONE_HUNDRED_MS_TO_MINUTES)/(RPM*Constants.TICKS_PER_REVOLUTION);
            //shooterMaster.config_kF(Constants.KSLOTIDX, targetVelocity_UnitsPer100ms);
          
            shooterMaster.set(ControlMode.PercentOutput, value);
        }
    }

    public static void stopShooter(){
        shooterMaster.set(0);
    }

    public static void turnTurret(double velocity) {
        turret.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.KTIMEOUTMS);
        turret.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.KTIMEOUTMS);
        turret.set(ControlMode.PercentOutput, 0.5* velocity);  
    }

    public static void BallPump(double rpm){
        ballPump.set(rpm);
    }
    public static void stopBallPump(){
        ballPump.set(0);
    }

    public static void Indexer(double rpm){
        indexer.set(rpm);
    }
    public static void stopIndexer(){
        indexer.set(0);
    }

}
