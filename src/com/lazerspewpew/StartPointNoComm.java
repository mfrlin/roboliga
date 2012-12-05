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
		int robotPower = 95;
		LineRobot crta = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S3, SensorPort.S2, robotPower);
		Button.waitForAnyPress();
		crta.followLine();
//		
//		WallRobot wall = new WallRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower);
//		LCD.clear();
//		LCD.drawString("wating for button press", 0, 0);
//		Button.waitForAnyPress();
//		wall.followWall();
	}
}
