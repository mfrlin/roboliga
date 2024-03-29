package com.lazerspewpew;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.comm.Bluetooth;
import lejos.util.Delay;

public class StartPoint {

	public static void main(String[] args) {
		/* gluh je ime bricka, ki mu NE dela ekrancek. uporabil ga bom za sledenje crte in kot receiverja, ker ga kot initiatorja ne morem,
		 * ker ne morem dodati drugega bricka na slepo pod known devices. pedro, ti bos verjetno uporabljal drug brick za sledenje zidu.
		 * uredi tale starting point in zakomentiraj bluetooth povezovanje. */
		
		if (Bluetooth.getFriendlyName().equals("Crta")) {
			
			int robotPower = 55;
			
			LineRobot lineFollower = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S3, SensorPort.S2, robotPower);
			lineFollower.actAsReceiver();
			LCD.clear();LCD.drawString("Click to START.", 0, 0);
			//Button.waitForAnyPress();
			lineFollower.followLine(); // tudi poslje signal na koncu
			lineFollower.rotateInPlace(100, 800);
			lineFollower.driveStraight(70, 1300); // verjetno se bomo morali bolj priblizati zidu
			lineFollower.rotateInPlace(100, 1600);
			Delay.msDelay(6000);
			lineFollower.driveStraight(70, 2600);
			lineFollower.follow();
			
		}
		
		
		else if (Bluetooth.getFriendlyName().equals("Zid")) {
			
			int robotPower = 80;
			WallRobot wallFollower = new WallRobot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S4, robotPower);
			LCD.drawString(Bluetooth.getFriendlyName() + ". Clk to connect.", 0, 0);
			Button.waitForAnyPress(); // pocakaj s pritiskom gumba, dokler drug robot ni odprt za connection
			LCD.clear();
			wallFollower.connectToRemote("Crta");
			LCD.clear();LCD.drawString("WAITING FOLLOW", 0, 0);
			wallFollower.follow();
			wallFollower.driveStraightUntilWall();
			wallFollower.followWall();
		}
	}


}

