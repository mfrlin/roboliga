package com.lazerspewpew;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.comm.Bluetooth;
import lejos.util.Delay;

public class StartPoint {

	public static void main(String[] args) {
		if (Bluetooth.getFriendlyName().equals("Crta")) {
			int robotPower = 95;
			LineRobot crta = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S2, SensorPort.S3, robotPower);
			crta.actAsReceiver();
			LCD.clear();
			LCD.drawString("Ready when you are.", 0, 0);
			Button.waitForAnyPress();
			LCD.clear();
			crta.followLine();
		}
		else if (Bluetooth.getFriendlyName().equals("Zid")) {
			int robotPower = 95;
			WallRobot zid = new WallRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower);
			LCD.drawString("Press and I shall connect.", 0, 0);
			Button.waitForAnyPress(); // pocakaj s pritiskom gumba, dokler drug robot ni odprt za connection
			LCD.clear();
			zid.connectToRemote("Crta");
			Delay.msDelay(2000);
			zid.follow();
			
		}
	}
}

