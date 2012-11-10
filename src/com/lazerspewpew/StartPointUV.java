package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class StartPointUV {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		int robotPower = 95;
		ActiveRobot wallFollower = new ActiveRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower, true);
//		wallFollower.setPID(false);
		LCD.drawString("Ready.", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		wallFollower.followWall();
//		wallFollower.followLine();
//		wallFollower.goStraight();
	}
}
