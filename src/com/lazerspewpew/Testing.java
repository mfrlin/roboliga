package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.util.Delay;

public class Testing {
	public static void main(String[] argv) {
		int[] powerArray = {40, 45, 50, 55, 60, 65, 70, 75, 80, 85};
		NXTMotor m1 = new NXTMotor(MotorPort.A);
		NXTMotor m2 = new NXTMotor(MotorPort.B);
		Button.waitForAnyPress();
		for (int power : powerArray) {
			m1.setPower(power);m2.setPower(power);
			m1.forward();m2.forward();
			Delay.msDelay(1000);
			m1.stop();m2.stop();
			LCD.drawInt(m1.getTachoCount(), 0, 1);
			LCD.drawInt(m2.getTachoCount(), 0, 2);
			m1.resetTachoCount();
			m2.resetTachoCount();
			Button.waitForAnyPress();
			LCD.clear();
		}
		
	}

}
