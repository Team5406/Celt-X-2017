package org.cafirst.frc.team5406.controller;

import org.cafirst.frc.team5406.util.Constants;

import edu.wpi.first.wpilibj.Joystick;

public class ControllerBase extends Joystick {
	
	//Number of buttons the controller has
	int numButtons;
	//The current states of the buttons. Starts at 1;
	boolean[] prevButtonStates;
	
	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public ControllerBase(int port) {
		super(port);
		numButtons = super.getButtonCount();
		prevButtonStates = new boolean[numButtons + 1];
	}
	
	/**Records the current states of the buttons on the controller.**/
	public void updateButtons()
	{
		for(int i = 0; i < prevButtonStates.length; i++)
			prevButtonStates[i] = getRawButton(i);
	}
	
	/**
	 * Checks to see if button is being pressed.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed.
	 */
	public boolean getButtonHeld(int button)
	{
		return super.getRawButton(button);
	}
	
	/**
	 * Checks to see if button has been pressed only once.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed and wasn't pressed before.
	 */
	public boolean getButtonOnce(int button)
	{
		return (super.getRawButton(button) && !prevButtonStates[button]);
	}
	
	/**
	 * Checks to see if button has been released.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed and wasn't pressed before.
	 */
	public boolean getButtonRelease(int button)
	{
		return (!super.getRawButton(button) && prevButtonStates[button]);
	}
	
	/**
	 * Applies the controller deadband to the desired value
	 * @param value The desired value
	 * @return 0 = not within deadband
	 */
	public static double applyDeadband(double value){
		return applyDeadband(value, Constants.CONTROLLER_DEADBAND);
	}
	
	/**
	 * Applies the desired deadband to the desired value
	 * @param value The desired value
	 * @param deadband The desire deadband
	 * @return 0 = not within deadband
	 */
	public static double applyDeadband(double value, double deadband){
		if ((-1*deadband) < value && value < deadband){
			return 0;
		}else{
			return value;
		}
	}

}
