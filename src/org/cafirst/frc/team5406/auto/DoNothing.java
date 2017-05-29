package org.cafirst.frc.team5406.auto;

public class DoNothing extends AutonomousRoutine {

	public DoNothing() {
		super("0 - Do Nothing");
		// TODO Auto-generated constructor stub
	}

	@Override
	public void init() {
		System.out.println("Doing Nothing on Porpose");

	}

	@Override
	public void periodic() {
		end();

	}

	@Override
	public void end() {
		// TODO Auto-generated method stub

	}
	
	@Override
	public AutonomousRoutine newInstance() {
		return new DoNothing();
	}

}
