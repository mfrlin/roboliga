package com.lazerspewpew;
import java.io.IOException;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class LineRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private int[] readingsArray = new int[5];
	private boolean lineEnd = false;
	//private long lastSend;
	//private long sendInterval = 500;
	
	public LineRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
	}
	
	public void steer(int difference) {
		if (difference >= 0) { // turn right or go straight
			leftMotor.setPower(maxPower);
			rightMotor.setPower(maxPower - difference);
		}
		else { // turn left
			leftMotor.setPower(maxPower + difference);
			rightMotor.setPower(maxPower);
		}
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		int arrayCounter = 0;
		int total = 0;
		readingsArray[arrayCounter % readingsArray.length] = rightReading + leftReading;
		for (int reading : readingsArray) {
			total += reading;
		}
		if (total < 50) { // this number needs tweaking
			lineEnd = true;
		}
		return rightReading - leftReading;
	}
	
	public void followLine() {
		int read, difference;
		long now, timeChange;
		
		while(true) {
			read = getSensorReadings(); 
			difference = (int)myPID.compute(read, 0);
			steer(difference);
			// pošiljanje podatkov
			/*now = System.currentTimeMillis();
			timeChange = now - lastSend;
			if (timeChange >= sendInterval) {
				sendTachoCounts();
				lastSend = now;
			}*/
			if (lineEnd) {
				try {
					outputStream.writeInt(1);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}

}
