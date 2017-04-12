package org.cafirst.frc.team5406.util;

import com.ctre.CANTalon;

public class cxCanTalon extends CANTalon {
  
  private double setPoint = 0.0;
  private double speedMultiplier = 1.0;
  
  public cxCanTalon(int deviceNumber){
    super(deviceNumber);
  }
  
  @Override
  public void set(double value){
    setPoint = value;
    super.set(setPoint * (this.getControlMode() == TalonControlMode.PercentVbus ? speedMultiplier : 1.0));
  }
  
  public double getSetpoint(){
    return setPoint;
  }
  
  public void setSpeedMultiplier(double value){
    speedMultiplier = value;
  }
}
