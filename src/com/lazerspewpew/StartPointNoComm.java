package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class StartPointNoComm {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		int robotPower = 100;
		LineRobot crta = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S3, SensorPort.S2, robotPower);
		Button.waitForAnyPress();
		crta.followLine();
	}
}
