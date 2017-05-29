package org.cafirst.frc.team5406.util;

import com.ctre.CANTalon;

public class Gearbox {

	private int encoderRatio;
	private int stallCurrent;
	private double freeCurrent;
	private int freeSpeed;
	private int gearing;
	private double target;
	private CANTalon.TalonControlMode mode;
	private boolean reverseEncoder;
	private double PID_F;
	private double PID_P;
	private double PID_I;
	private double PID_D;
	private boolean invert;
	
	public int getEncoderRatio() {
		return encoderRatio;
	}
	public void setEncoderRatio(int encoderRatio) {
		this.encoderRatio = encoderRatio;
	}
	public int getStallCurrent() {
		return stallCurrent;
	}
	public void setStallCurrent(int stallCurrent) {
		this.stallCurrent = stallCurrent;
	}
	public double getFreeCurrent() {
		return freeCurrent;
	}
	public void setFreeCurrent(double freeCurrent) {
		this.freeCurrent = freeCurrent;
	}
	public int getFreeSpeed() {
		return freeSpeed;
	}
	public void setFreeSpeed(int freeSpeed) {
		this.freeSpeed = freeSpeed;
	}
	public int getGearing() {
		return gearing;
	}
	public void setGearing(int gearing) {
		this.gearing = gearing;
	}
	public double getTarget() {
		return target;
	}
	public void setTarget(double target) {
		this.target = target;
	}
	public CANTalon.TalonControlMode getMode() {
		return mode;
	}
	public void setMode(CANTalon.TalonControlMode mode) {
		this.mode = mode;
	}
	public boolean isReverseEncoder() {
		return reverseEncoder;
	}
	public void setReverseEncoder(boolean reverseEncoder) {
		this.reverseEncoder = reverseEncoder;
	}
	public double getPID_F() {
		return PID_F;
	}
	public void setPID_F(double pID_F) {
		PID_F = pID_F;
	}
	public double getPID_P() {
		return PID_P;
	}
	public void setPID_P(double pID_P) {
		PID_P = pID_P;
	}
	public double getPID_I() {
		return PID_I;
	}
	public void setPID_I(double pID_I) {
		PID_I = pID_I;
	}
	public double getPID_D() {
		return PID_D;
	}
	public void setPID_D(double pID_D) {
		PID_D = pID_D;
	}
	public boolean isInvert() {
		return invert;
	}
	public void setInvert(boolean invert) {
		this.invert = invert;
	}
}
