package org.cafirst.frc.team5406.util;

import com.ctre.CANTalon;

public class cxCanTalon extends CANTalon {
  
  private double setPoint = 0.0;
  private double speedMultiplier = 1.0;
  private boolean enableCurrentProtection = false;
  
  public cxCanTalon(int deviceNumber){
    super(deviceNumber);
  }
  
  public void enableCurrentProtection(boolean enable){
    enableCurrentProtection = enable;
  }
  
  @Override
  public void set(double value){
    setPoint = value;
    value *= (enableCurrentProtection && this.getControlMode() == TalonControlMode.PercentVbus) ? speedMultiplier : 1.0;
    super.set(value);
  }
  
  public double getSetpoint(){
    return setPoint;
  }
  
  public void setSpeedMultiplier(double value){
    speedMultiplier = value;
  }
}
