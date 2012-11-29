package com.lazerspewpew;
import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
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
		calibrateLightSensors(); // calibrira, in nastavi delovanje na staticno normalizacijo
		Button.waitForAnyPress(); // FIXME: preveri ali se pravilno kalibrira. Nato izbrisi;
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
	}

	private void calibrateLightSensors() {
		int boundingMaxLeft = 0;
		int boundingMinLeft = 9999;
		int boundingMaxRight = 0;
		int boundingMinRight = 9999;

		int stMeritev = 10;

		LCD.drawString("Postavi na crno crto", 0, 0);
		Button.waitForAnyPress();
		LCD.drawString("Meljem podatke", 0, 1);
		for (int i = 0; i < stMeritev; i++) {
			int rawValueLeft = leftSensor.getLightValue();
			int rawValueRight = rightSensor.getLightValue();
			if(rawValueLeft < boundingMinLeft)
				boundingMinLeft = rawValueLeft;
			if(rawValueRight < boundingMinRight)
				boundingMinRight = rawValueRight;
			//boundingMinRight = (rawValueRight > boundingMinRight) ? rawValueRight : boundingMinRight;
		}
		Sound.beep();

		LCD.clear();

		LCD.drawString("Postavi na belo poglago", 0, 0);
		Button.waitForAnyPress();
		LCD.drawString("Meljem podatke", 0, 1);
		for (int i = 0; i < stMeritev; i++) {
			int rawValueLeft = leftSensor.getLightValue();
			int rawValueRight = rightSensor.getLightValue();
			if(rawValueLeft > boundingMaxLeft)
				boundingMaxLeft = rawValueLeft;
			if(rawValueRight > boundingMaxRight)
				boundingMaxRight = rawValueRight;
		}
		Sound.beep();

		LCD.clear();
		LCD.drawString("Left: [" + boundingMinLeft +", "+boundingMaxLeft+"]", 0, 0);
		LCD.drawString("Right: [" + boundingMinRight +", "+boundingMaxRight+"]", 0, 1);
		
		leftSensor.setFixedBoundaries(boundingMinLeft, boundingMaxLeft);
		rightSensor.setFixedBoundaries(boundingMinRight, boundingMaxRight);
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
