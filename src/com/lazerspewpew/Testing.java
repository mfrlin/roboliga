package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.util.Delay;

public class Testing {
	public static void main(String[] argv) {
		int[] powerArray = {30, 40, 50, 60, 70, 75, 80, 85, 90, 95, 100};
		NXTMotor m1 = new NXTMotor(MotorPort.A);
		NXTMotor m2 = new NXTMotor(MotorPort.B);
		NXTMotor m3 = new NXTMotor(MotorPort.C);
		for (int power : powerArray) {
			m1.setPower(power);m2.setPower(power);m3.setPower(power);
			m1.forward();m2.forward();m3.forward();
			Delay.msDelay(5000);
			m1.stop();m2.stop();m3.stop();
			LCD.drawInt(m1.getTachoCount(), 0, 1);
			LCD.drawInt(m2.getTachoCount(), 0, 2);
			LCD.drawInt(m3.getTachoCount(), 0, 3);
			Button.waitForAnyPress();
			LCD.clear();
		}
	}

}
