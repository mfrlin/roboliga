package com.lazerspewpew;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;


public class Motor extends NXTMotor{
	
	private double maxPowerOverSpeed; /* Should NEVER be larger than 1 */

	public Motor(MotorPort port, double maxPowerOverSpeed) {
		super(port, lejos.nxt.BasicMotorPort.PWM_BRAKE);
		this.maxPowerOverSpeed = maxPowerOverSpeed;
	}
	
	public void empower(int power) {
		
		power = (int) (power * maxPowerOverSpeed);
		
//		if (maxPowerOverSpeed < 1) {
//			LCD.clear();
//			LCD.drawInt(power, 0, 0);
//		}
		
		if (power > 100) {
			power = 100;
		} else if (power < -100) {
			power = -100;
		}
		
		this.setPower(power);
		
	}
}
