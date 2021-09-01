package frc;

public final class Constants {
    public static final int LEFT_DRIVE_MOTOR_ONE = 1; //TalonSRX
    public static final int LEFT_DRIVE_MOTOR_TWO = 2; //TalonSRX
    public static final int LEFT_DRIVE_MOTOR_THREE = 3; //TalonSRX
    public static final int RIGHT_DRIVE_MOTOR_ONE = 4; //TalonSRX
    public static final int RIGHT_DRIVE_MOTOR_TWO = 5; //TalonSRX
    public static final int RIGHT_DRIVE_MOTOR_THREE = 6; //TalonSRX

    public static final int SHOOTER_WHEEL_MOTOR_ONE = 10; // TalonSRX
    public static final int SHOOTER_WHEEL_MOTOR_TWO = 11; // TalonSRX

    public static final int TURRET_MOTOR = 12; // TalonSRX

    public static final int INTAKE_MOTOR = 7; // TalonSRX

    public static final int BALL_PUMP_MOTOR = 8;

    public static final int INDEXER_MOTOR = 9;

    public static final int DRIVE_TRAIN_CURRENT = 30; //Amps
    public static final int INTAKE_CURRENT = 20;
    public static final int BALL_PUMP_CURRENT = 10;
    public static final int INDEXER_CURRENT = 10;
    public static final int SHOOTER_CURRENT = 20;
    public static final int TURRET_CURRENT = 5;
    

    public static final int INTAKE_TARGET_SPEED = 1200;
    public static final double OUTAKE_BALLS = -0.5;
    public static final int SHOOTER_RPM = 5800;


    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;



    public static final int KSLOTIDX = 0;

    public static final double SHOOTER_PID_F = 0.0132;
    public static final double SHOOTER_PID_P = 0.04;
    public static final double SHOOTER_PID_I = 0.0000;
    public static final double SHOOTER_PID_D = 0.006;

    public static final double TURRET_PID_F = 0.3;
    public static final double TURRET_PID_P = 6;
    public static final double TURRET_PID_I = 0.0;
    public static final double TURRET_PID_D = 5
    ;


    public static final double SHOOTER_SCALING_FACTOR = 0.5;
    public static final double NUMBER_OF_BUCKETS = 1023;
    public static final double ONE_HUNDRED_MS_TO_MINUTES = 600;
    public static final double TICKS_PER_REVOLUTION = 4096;

    public final static int KTIMEOUTMS = 30;
}