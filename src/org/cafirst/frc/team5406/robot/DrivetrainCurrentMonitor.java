package org.cafirst.frc.team5406.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.util.Motors;

import java.util.ArrayList;
import java.util.List;

/**
 * Created based off the paper found here:
 * https://www.chiefdelphi.com/media/papers/3364
 *
 * @author kestin
 */
public class DrivetrainCurrentMonitor {
  
  public static final double MIN_VOLTAGE = 8.0;// V
  public static double BATTERY_VOLTAGE = 12.5;
  public static double BATTERY_RESISTANCE = 0.012;
  
  private PowerDistributionPanel pdp;
  
  private Drive drive;
  private Motors.Motor motorType;
  private int motorsPerSide;
  private double lastScalingFactor;
  
  public DrivetrainCurrentMonitor(Drive drive, Motors.Motor motorType, int motorsPerSide){
    this.drive = drive;
    this.motorType = motorType;
    this.motorsPerSide = motorsPerSide;
    
    lastScalingFactor = 1.0;
    pdp = new PowerDistributionPanel(1);
    updateBatteryVotlage();
  }
  
  public void updateBatteryVotlage(){
    BATTERY_VOLTAGE = pdp.getVoltage();
  }
  
  private double getEncRatio(){
    return 1.0 / 4096 * 2 * Math.PI * (drive.isShiftedHigh() ? Drive.HIGH_GEAR_RATIO : Drive.LOW_GEAR_RATIO);
  }
  
  public double getEstimatedCurrent(){
    double leftCurrent = motorsPerSide * motorType.getEstimatedCurrent(getLeftVoltage(), getLeftSpeed());
    double rightCurrent = motorsPerSide * motorType.getEstimatedCurrent(getRightVoltage(), getRightSpeed());
    
    return leftCurrent + rightCurrent;
  }
  
  public double getEstimatedVoltage(){
    return BATTERY_VOLTAGE - BATTERY_RESISTANCE * getEstimatedCurrent();
  }
  
  private double getLeftSpeed(){
    return drive.getLeftEncVel() * getEncRatio();
  }
  
  private double getRightSpeed(){
    return drive.getRightEncVel() * getEncRatio();
  }
  
  private double getLeftVoltage(){
    return BATTERY_VOLTAGE * drive.getLeftSetpoint();
  }
  
  private double getRightVoltage(){
    return BATTERY_VOLTAGE * drive.getRightSetpoint();
  }
  
  public double getScalingFactor(){
    double leftSpeed = getLeftSpeed();
    double leftVoltage = getLeftVoltage();
    double rightSpeed = getRightSpeed();
    double rightVoltage = getRightVoltage();
    
    List<Double> gammaVals = new ArrayList<>();
    for(int i = 0; i < 2; i++){
      for(int j = 0; j < 2; j++){
        double val1 = (motorType.R_m * (BATTERY_VOLTAGE - MIN_VOLTAGE) /
                (motorsPerSide * BATTERY_RESISTANCE)) * (i == 0 ? 1.0 : -1.0);
        double val2 = (motorType.K_i * rightSpeed) + (motorType.K_i * leftSpeed) * (j == 0 ? 1.0 : -1.0);
        double val3 = rightVoltage + leftVoltage * (j == 0 ? 1.0 : -1.0);
        gammaVals.add((val1 + val2) / val3);
      }
    }
    
    if(getEstimatedVoltage() < MIN_VOLTAGE){
      double gammaVal = -1.0;
      for(double val : gammaVals){
        if(val >= 0 && val <= 1.0 && val > gammaVal){
          gammaVal = val;
        }
      }
      if(gammaVal == -1)
        gammaVal = 1.0;
      this.lastScalingFactor = gammaVal;
      return gammaVal;
    }
    this.lastScalingFactor = 1.0;
    return 1.0;
    
  }
  
  public void logSmartDash(){
    SmartDashboard.putNumber("gamma", lastScalingFactor);
    SmartDashboard.putNumber("est_current", getEstimatedCurrent());
    SmartDashboard.putNumber("est_voltage", getEstimatedVoltage());
    SmartDashboard.putNumber("left_w", getLeftSpeed());
    SmartDashboard.putNumber("right_w", getRightSpeed());
  }
  
}
