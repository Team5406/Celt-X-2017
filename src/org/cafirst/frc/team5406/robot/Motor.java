package org.cafirst.frc.team5406.robot;

public class Motor
{
    public int id;
    public boolean master;
    public int follow;
    
    
    public Motor(int id, boolean master, int follow) {
    	this.id = id;
    	this.master = master;
    	this.follow = follow;
    }
}