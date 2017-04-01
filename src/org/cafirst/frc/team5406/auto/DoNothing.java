package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.auto.AutonomousRoutine;


public class DoNothing extends AutonomousRoutine{

	public DoNothing() {
		super("Do Nothing");
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
		
	}

}
