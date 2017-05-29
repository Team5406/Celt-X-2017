package org.cafirst.frc.team5406.controller;

public class XboxController extends ControllerBase {

	public static enum XboxButton
	{
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4),
		LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(7), START_BUTTON(8),
		LEFT_STICK(9), RIGHT_STICK(10),
		
		LEFT_X_AXIS(0), LEFT_Y_AXIS(1),
		LEFT_TRIGGER_AXIS(2), RIGHT_TRIGGER_AXIS(3),
		RIGHT_X_AXIS(4), RIGHT_Y_AXIS(5);
		
		private int value;
		
		XboxButton(int value)
		{
			this.value = value;
		}
		
		/**@return The integer value of the button.*/
		public int getValue() { return value; }
	}
	
	public static enum DirectionPad{
		UP,
		DOWN,
		LEFT,
		RIGHT,
		NONE
	}
	
	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public XboxController(int port) {
		super(port);
	}
	
	/**
	 * Checks to see if button is being pressed.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed.
	 */
	public boolean getButtonHeld(XboxButton button)
	{
		return super.getButtonHeld(button.getValue());
	}
	
	/**
	 * Checks to see if button has been pressed only once.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed and wasn't pressed before.
	 */
	public boolean getButtonOnce(XboxButton button)
	{
		return super.getButtonOnce(button.getValue());
	}
	
	/**
	 * Checks to see if button has been released.
	 * @param button The desired button you want to check.
	 * @return True if button is being pressed and wasn't pressed before.
	 */
	public boolean getButtonRelease(XboxButton button)
	{
		return super.getButtonRelease(button.getValue());
	}

	/**
	 * @return Left X axis value.
	 */
	public double getLeftX()
	{
		return super.getRawAxis(XboxButton.LEFT_X_AXIS.getValue());
	}
	
	/**
	 * @return Left Y axis value.
	 */
	public double getLeftY()
	{
		return super.getRawAxis(XboxButton.LEFT_Y_AXIS.getValue());
	}
	
	/**
	 * @return Right X axis value.
	 */
	public double getRightX()
	{
		return super.getRawAxis(XboxButton.RIGHT_X_AXIS.getValue());
	}
	
	/**
	 * @return Right Y axis value.
	 */
	public double getRightY()
	{
		return super.getRawAxis(XboxButton.RIGHT_Y_AXIS.getValue());
	}
	
	/**
	 * @return Left Trigger value.
	 */
	public double getLeftTrigger(){
		return super.getRawAxis(XboxButton.LEFT_TRIGGER_AXIS.getValue());
	}
	
	/**
	 * @return True if Left Trigger is pressed
	 */
	public boolean getLeftTriggerPressed(){
		return Math.abs(super.getRawAxis(XboxButton.LEFT_TRIGGER_AXIS.getValue())) > org.cafirst.frc.team5406.util.Constants.CONTROLLER_DEADBAND;
	}
	
	/**
	 * @return Right Trigger value.
	 */
	public double getRightTrigger(){
		return super.getRawAxis(XboxButton.RIGHT_TRIGGER_AXIS.getValue());
	}
	
	/**
	 * @return True if Right Trigger is pressed
	 */
	public boolean getRightTriggerPressed(){
		return Math.abs(super.getRawAxis(XboxButton.RIGHT_TRIGGER_AXIS.getValue())) > org.cafirst.frc.team5406.util.Constants.CONTROLLER_DEADBAND;
	}
	
	/**
	 * Gets the current direction of the direction pad.
	 * @return Degree value of the direction pad.
	 */
	public DirectionPad getDirectionPad(){
		int dir = super.getPOV();
		switch(dir){
			default:
				return DirectionPad.NONE;
			case 0:
				return DirectionPad.UP;
			case 90:
				return DirectionPad.RIGHT;
			case 180:
				return DirectionPad.DOWN;
			case 270:
				return DirectionPad.LEFT;
		}
	}
}
