package org.cafirst.frc.team5406.robot;
import org.cafirst.frc.team5406.robot.Motor;
import org.cafirst.frc.team5406.robot.Gearbox;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

public final class Constants {
	
	public static AHRS navX = new AHRS(SPI.Port.kMXP);
	
	public static boolean IS_PRACTICE_BOT = false;
	
	//centerX is not a constant - it will be changed dynamically
	public static double centerX = 0;
	public static int REVLimit = Integer.MIN_VALUE;
	public static int FWDLimit = Integer.MAX_VALUE;
	
	public static int PRACTICE_BOT_LEFT_HOPPER_TURRET_START = -270; //degrees
	public static int COMP_BOT_LEFT_HOPPER_TURRET_START = -220; //degrees
	public static int PRACTICE_BOT_RIGHT_HOPPER_TURRET_START = -70; //degrees
	public static int COMP_BOT_RIGHT_HOPPER_TURRET_START = -70; //degrees
	
	//drive constants - units in inches
	public static int WHEEL_TRACK = 34;
	public static int ROBOT_LENGTH = 36;
	
	public static int WHEEL_DIAM = 4;
	public static int MAX_VELOCITY_LOW = 120; //(in/s)
	public static int MAX_ACCEL_LOW = 90; //(in/s^2)
	public static double COEFF_FRICTION = 1.0;
	public static int LOW_GEAR_RATIO = 9;
	public static int HIGH_GEAR_RATIO = 4;
	public static int MAX_VELOCITY_HIGH = 240;
	public static int MAX_ACCEL_HIGH = 90;
	
	/*public static double GYRO_PID_P = 0.025;
	public static double GYRO_PID_I = 0.002;
	public static double GYRO_PID_D = 0.009;*/
	
	/*public static double GYRO_PID_P = 0.0025; //0.021
	public static double GYRO_PID_I = 0.000;
	public static double GYRO_PID_D = 0.001;*/

	public static double GYRO_PID_P = 0.025;
	public static double GYRO_PID_I = 0.000;
	public static double GYRO_PID_D = 0.125;
	
	public static int TURRET_ROTATION_TICKS = 29580;
	public static int TURRET_ROTATION_DEG = 309;
	public static int AXIS_FOV = 63;
	public static double CAMERA_OFFSET = 2.5;

	
	public static String AXIS_IP = "10.54.6.17";
	
	public static int IMAGE_WIDTH = 480;
	public static int IMAGE_HEIGHT = 320;
	public static int OFF_CENTER = 0;
	public static int IMAGE_CENTER = IMAGE_WIDTH / 2 + OFF_CENTER;
	
	public static double CAMERA_FIELD = Math.PI/3;
	public static double CAMERA_FIELD_HALF = CAMERA_FIELD/2;
	
	public static int INDEXER_SPEED = 900;
	

	
	public static boolean MASTER = true;
	public static boolean SLAVE = false;
	
	public static double xboxControllerDeadband = 0.2;
	
	public static Gearbox INTAKE = new Gearbox();
	public static Gearbox LEFT_DRIVE = new Gearbox();
	public static Gearbox RIGHT_DRIVE = new Gearbox();
	public static Gearbox BALL_PUMP = new Gearbox();
	public static Gearbox INDEXER = new Gearbox();
	public static Gearbox SHOOTER = new Gearbox();
	public static Gearbox TURRET = new Gearbox();
	public static Gearbox CLIMBER = new Gearbox();
	
	public static DoubleSolenoid.Value SHIFT_HIGH = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value SHIFT_LOW = DoubleSolenoid.Value.kReverse;
	public static DoubleSolenoid.Value HOOD_UP = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value HOOD_DOWN = DoubleSolenoid.Value.kReverse;
	public static DoubleSolenoid.Value GEAR_LIFT_DOWN = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value GEAR_LIFT_UP = DoubleSolenoid.Value.kReverse;
	public static DoubleSolenoid.Value GEAR_GRIP_CLOSED = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value GEAR_GRIP_OPEN = DoubleSolenoid.Value.kReverse;
	
	//PCM Ports
	public static int HOOD_FORWARD = 3;
	public static int HOOD_REVERSE = 2;
	public static int GEAR_GRIP_FORWARD = 5;
	public static int GEAR_GRIP_REVERSE = 4;
	public static int GEAR_LIFT_FORWARD = 6;
	public static int GEAR_LIFT_REVERSE = 7;
	public static int SHIFT_FORWARD = 0; 
	public static int SHIFT_REVERSE = 1;
	
	
	//DIO Ports
	public static int CCW_LIMIT_SWITCH = 0;
	public static int CW_LIMIT_SWITCH = 1;
	public static int GEAR_SWITCH = 2;
	public static int GEAR_LIGHT = 4;
	public static int PRACTICE_BOT = 9; //jumper to short pins on practice bot
	
	
	public boolean equalsDeadband(double value){
	return ((-1*xboxControllerDeadband) < value && value < xboxControllerDeadband);
	}
	
	
	public Constants() {
		INTAKE.encoder_ratio=1;
		INTAKE.stall_current = 89;
		INTAKE.free_current=3;
		INTAKE.free_speed=5840; //free_speed
		INTAKE.gearing=4; //gearing
		INTAKE.target=1200; //rpm
		INTAKE.reverse_encoder = true; //reverse_encoder
		//INTAKE.mode = CANTalon.TalonControlMode.Speed;
		INTAKE.mode = CANTalon.TalonControlMode.PercentVbus;
		INTAKE.invert = false;
		INTAKE.PID_F=(1023*600)/(INTAKE.target*4096); //PID_F
		INTAKE.PID_P=0.05*1023; //PID_P
		INTAKE.PID_I=0.00; //PID_I
		INTAKE.PID_D=10*INTAKE.PID_P; //PID_D
		INTAKE.motors= new Motor[] {
				new Motor(7, MASTER, 0),
		};
		
		LEFT_DRIVE.encoder_ratio=1;
		LEFT_DRIVE.stall_current = 131;
		LEFT_DRIVE.free_current=2.7;
		LEFT_DRIVE.free_speed=5330; //free_speed
		LEFT_DRIVE.gearing=9; //gearing
		LEFT_DRIVE.target=600; //rpm
		LEFT_DRIVE.mode = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
		LEFT_DRIVE.invert = true;
		LEFT_DRIVE.reverse_encoder = true; //reverse_encoder
		LEFT_DRIVE.PID_F=-0.3;//-1*(1023*600)/(LEFT_DRIVE.target*4096); //PID_F
		/*LEFT_DRIVE.PID_P=0.0062; //PID_P
		LEFT_DRIVE.PID_I=0.00000; //PID_I
		LEFT_DRIVE.PID_D=0.0003; //PID_D*/
		LEFT_DRIVE.PID_P=0.008; //PID_P
		LEFT_DRIVE.PID_I=0.00000; //PID_I
		LEFT_DRIVE.PID_D=0.0003; //PID_D
		LEFT_DRIVE.motors= new Motor[] {
				new Motor(1, MASTER, 0),
				new Motor(2, SLAVE, 1),
				new Motor(3, SLAVE, 1)
		};

		RIGHT_DRIVE.encoder_ratio=1;
		RIGHT_DRIVE.stall_current = 131;
		RIGHT_DRIVE.free_current=2.7;
		RIGHT_DRIVE.free_speed=5330; //free_speed
		RIGHT_DRIVE.gearing=9; //gearing
		RIGHT_DRIVE.target=600; //rpm
		RIGHT_DRIVE.mode = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
		RIGHT_DRIVE.reverse_encoder = false; //reverse_encoder
		RIGHT_DRIVE.invert = false;
		RIGHT_DRIVE.PID_F=0.3;//(1023*600)/(RIGHT_DRIVE.target*4096); //PID_F
		RIGHT_DRIVE.PID_P=0.008; //PID_P
		RIGHT_DRIVE.PID_I=0.00000; //PID_I
		RIGHT_DRIVE.PID_D=0.0003; //PID_D
		/*RIGHT_DRIVE.PID_P=0.0062; //PID_P
		RIGHT_DRIVE.PID_I=0.00000; //PID_I
		RIGHT_DRIVE.PID_D=0.0003; //PID_D*/
		RIGHT_DRIVE.motors= new Motor[] {
				new Motor(4, MASTER, 0),
				new Motor(5, SLAVE, 4),
				new Motor(6, SLAVE, 4)
		};
		
		BALL_PUMP.encoder_ratio=1;
		BALL_PUMP.stall_current = 89;
		BALL_PUMP.free_current=3;
		BALL_PUMP.free_speed=5840; //free_speed
		BALL_PUMP.gearing=7; //gearing
		BALL_PUMP.target=500; //rpm
		BALL_PUMP.mode = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
		BALL_PUMP.reverse_encoder = false; //reverse_encoder
		BALL_PUMP.invert = false;
		BALL_PUMP.PID_F=(1023*600)/(BALL_PUMP.target*4096); //PID_F
		BALL_PUMP.PID_P=0.05*1023; //PID_P
		BALL_PUMP.PID_I=0.00; //PID_I
		BALL_PUMP.PID_D=10*BALL_PUMP.PID_P; //PID_D
		BALL_PUMP.motors= new Motor[] {
				new Motor(8, MASTER, 0),
		};
		
		INDEXER.encoder_ratio=1;
		INDEXER.stall_current = 41;
		INDEXER.free_current=1.8;
		INDEXER.free_speed=13180; //free_speed
		INDEXER.gearing=10; //gearing
		INDEXER.target=865; //rpm
		INDEXER.mode = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
		INDEXER.reverse_encoder = false; //reverse_encoder
		INDEXER.invert = false;
		INDEXER.PID_F=-1*(1023*600)/(INDEXER.target*4096); //PID_F
		INDEXER.PID_P=0.03; //PID_P
		INDEXER.PID_I=0.00; //PID_I
		INDEXER.PID_D=10*INDEXER.PID_P; //PID_D
		INDEXER.motors= new Motor[] {
				new Motor(9, MASTER, 0),
		};

		
		SHOOTER.encoder_ratio=1;
		SHOOTER.stall_current = 134;
		SHOOTER.free_current=0.7;
		SHOOTER.free_speed=18730; //free_speed
		SHOOTER.gearing=1; //gearing
		SHOOTER.target=5700; //rpm
		SHOOTER.mode = CANTalon.TalonControlMode.Speed;
		SHOOTER.reverse_encoder = false; //reverse_encoder
		SHOOTER.invert = false;
		SHOOTER.PID_F=0.0132; //PID_F
		SHOOTER.PID_P=0.04; //PID_P //PracticeBOT: 0.06, CompBot: 0.04
		SHOOTER.PID_I=0.0000; //PID_I
		SHOOTER.PID_D=0.006; //PID_D
		/*SHOOTER.PID_F=0.015; //PID_F
		SHOOTER.PID_P=0.035; //PID_P
		SHOOTER.PID_I=0; //PID_I
		SHOOTER.PID_D=0.0002; //PID_D*/
		SHOOTER.motors= new Motor[] {
				new Motor(10, MASTER, 0),
				new Motor(11, SLAVE, 10),
		};
		
		//need to set position instead of RPM.
		TURRET.encoder_ratio=1;
		TURRET.stall_current = 41;
		TURRET.free_current=1.8;
		TURRET.free_speed=13180; //free_speed
		TURRET.gearing=10; //gearing
		TURRET.target=100; //rpm
		TURRET.mode = CANTalon.TalonControlMode.MotionMagic;
		TURRET.reverse_encoder = false; //reverse_encoder
		TURRET.invert = false;
		TURRET.PID_F=0.3; //PID_F
		TURRET.PID_P=6; //PID_P
		TURRET.PID_I=0.0; //PID_I
		TURRET.PID_D=5; //PID_D
		TURRET.motors= new Motor[] {
				new Motor(12, MASTER, 0),
		};
		
		CLIMBER.encoder_ratio=1;
		CLIMBER.stall_current = 41;
		CLIMBER.free_current=1.8;
		CLIMBER.free_speed=13180; //free_speed
		CLIMBER.gearing=45; //gearing
		CLIMBER.target=100; //rpm
		CLIMBER.mode = CANTalon.TalonControlMode.PercentVbus;//CANTalon.TalonControlMode.Speed;
		CLIMBER.reverse_encoder = false; //reverse_encoder
		CLIMBER.invert = false;
		CLIMBER.PID_F=(1023*600)/(CLIMBER.target*4096); //PID_F
		CLIMBER.PID_P=0.05*1023; //PID_P
		CLIMBER.PID_I=0.00; //PID_I
		CLIMBER.PID_D=10*CLIMBER.PID_P; //PID_D
		CLIMBER.motors= new Motor[] {
				new Motor(13, MASTER, 0),
				new Motor(14, SLAVE, 13),
		};
	}
}