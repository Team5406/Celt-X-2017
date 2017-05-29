package org.cafirst.frc.team5406.auto;

public abstract class AutonomousRoutine {
	
	private String name;
	private boolean isRunning;
	
	public AutonomousRoutine(String name)
	{
		this.name = name;
		isRunning = false;
	}
	
	/**Initializes auto values*/
	public void start()
	{
		isRunning = true;
		init();
	}
	
	/**Runs the auto*/
	public void run()
	{
		periodic();
	}
	
	/**Stops and ends the auto*/
	public void stop()
	{
		isRunning = false;
		end();
	}
	
	public String getName() { return name; }
	public boolean isRunning() { return isRunning; }
	
	public abstract void init();
	public abstract void periodic();
	public abstract void end();
	
	/**
	 * Clones the auto to reset values
	 * @return The cloned routine
	 */
	public abstract AutonomousRoutine newInstance();

}
