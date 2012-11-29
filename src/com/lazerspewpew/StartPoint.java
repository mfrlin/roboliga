package com.lazerspewpew;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.comm.Bluetooth;
import lejos.util.Delay;

public class StartPoint {

	public static void main(String[] args) {
		/* gluh je ime bricka, ki mu NE dela ekrancek. uporabil ga bom za sledenje crte in kot receiverja, ker ga kot initiatorja ne morem,
		 * ker ne morem dodati drugega bricka na slepo pod known devices. pedro, ti bos verjetno uporabljal drug brick za sledenje zidu.
		 * uredi tale starting point in zakomentiraj bluetooth povezovanje. */
		
		if (Bluetooth.getFriendlyName().equals("Crta")) {
			
			int robotPower = 95;
			
			LineRobot lineFollower = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S2, SensorPort.S3, robotPower);
			lineFollower.actAsReceiver();
			LCD.drawString("Ready.", 0, 0);
			Button.waitForAnyPress();
			LCD.clear();
			lineFollower.followLine();
		}
		
		else if (Bluetooth.getFriendlyName().equals("Zid")) {
			
			int robotPower = 95;
			
			WallRobot wallFollower = new WallRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower);
			LCD.drawString(Bluetooth.getFriendlyName() + ". Press to connect.", 0, 0);
			Button.waitForAnyPress(); // pocakaj s pritiskom gumba, dokler drug robot ni odprt za connection
			LCD.clear();
			wallFollower.connectToRemote("Crta");
			wallFollower.followWall();
		}
	}
}

