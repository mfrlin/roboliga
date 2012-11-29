package com.lazerspewpew;

import java.io.IOException;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class WallRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private UltrasonicSensor usFrontSensor;
	private UltrasonicSensor usRightSensor;
	private PID myPID;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallFrontDistance; /* Kdaj reagira ko zazna zid spredaj */
	
	public WallRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort); 
		wantedWallDistance = 15; // na kaki razdalji se naj pelje desno
		wallFrontDistance = 20; // na kaki razdalji naj zaène upoštevati sprednji senzor
		
		usFrontSensor = new UltrasonicSensor(frontSensorPort);
		usRightSensor = new UltrasonicSensor(rightSensorPort);
		usFrontSensor.continuous(); usRightSensor.continuous();
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
	}
	
	public void steer(int difference) {
		if (difference <= 0) { // turn right or go straight
			leftMotor.setPower(maxPower);
			rightMotor.setPower(maxPower + difference);
		}
		else { // turn left
			leftMotor.setPower(maxPower - difference);
			rightMotor.setPower(maxPower);
		}
	}
	
	public int getSensorReadings() {
		int frontReading = usFrontSensor.getDistance();
		int rightReading = usRightSensor.getDistance();
		if (rightReading > 20) { // indicates right turn, because we get large right reading so we steer right
			return rightReading;
		}
		else if (frontReading <= wallFrontDistance ) { // indicates left turn because we have wall infront
			return rightReading - frontReading;
		}
		else {
			return rightReading;
		}
	}
	
	public void followWall() {
		int read, difference;
		long now, timeChange;
		try {
			int startSignal = inputStream.readInt();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		while(true) {
			read = getSensorReadings(); 
			difference = (int)myPID.compute(read, wantedWallDistance);
			steer(difference);
			// pošiljanje podatkov
			/*now = System.currentTimeMillis();
			timeChange = now - lastSend;
			if (timeChange >= sendInterval) {
				sendTachoCounts();
				lastSend = now;
			}*/
		}
	}
}
