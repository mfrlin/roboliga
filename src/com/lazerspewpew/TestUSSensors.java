package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;

public class TestUSSensors {
	
	/**
	 * @param args
	 * It takes about 34ms for each sensor reading. 
	 */
	public static void main(String[] args) {
		int robotPower = 95;
		ActiveRobot wallFollower = new ActiveRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower, true);
		LCD.drawString("Ready.", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		long start = System.currentTimeMillis();
		for (int i = 0; i < 100; i++) {
			wallFollower.usFrontSensor.getDistance();
			wallFollower.usRightSensor.getDistance();
//			Sound.beep();
		}
		long end = System.currentTimeMillis();
		LCD.drawInt((int) (end - start), 0, 0);
		LCD.drawInt((int) ((end - start)/100.0), 0, 1);
		Button.waitForAnyPress();
//		wallFollower.followWall();
//		wallFollower.goStraight();
	}

}
