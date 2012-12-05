package com.lazerspewpew;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class Testing {
	public static void main(String[] argv) {
		int robotPower = 55;
		LineRobot lineFollower = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S3, SensorPort.S2, robotPower);
		lineFollower.rotateInPlace(100, 800);
		Button.waitForAnyPress();
		lineFollower.driveStraight(70, 1400); // verjetno se bomo morali bolj priblizati zidu
		Button.waitForAnyPress();
		lineFollower.rotateInPlace(100, 1600);
	}

}
