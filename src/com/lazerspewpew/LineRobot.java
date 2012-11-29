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
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
		calibrateLightSensors(); // calibrira, in nastavi delovanje na staticno normalizacijo
		Button.waitForAnyPress(); // FIXME: preveri ali se pravilno kalibrira. Nato izbrisi;
	}

	private void calibrateLightSensors() {
		int boundingMaxLeft = 0;
		int boundingMinLeft = 9999;
		int boundingMaxRight = 0;
		int boundingMinRight = 9999;

		

		LCD.clear();
		LCD.drawString("Postavi me pred crto", 0, 0);
		Button.waitForAnyPress();
		steer(0);
		int stMeritev = 100, leftTemp, rightTemp;
		//int[] arr = new int[stMeritev];
		leftTemp = leftSensor.getLightValue();
		rightTemp = rightSensor.getLightValue();
		boundingMinLeft = leftTemp;
		boundingMaxLeft = leftTemp;
		boundingMinRight = rightTemp;
		boundingMaxRight = rightTemp;
		for(int i=0;i<stMeritev;i++){
			Delay.msDelay(5);
			leftTemp = leftSensor.getLightValue();
			rightTemp = rightSensor.getLightValue();
			boundingMinLeft = (leftTemp < boundingMinLeft) ? leftTemp : boundingMinLeft;
			boundingMaxLeft = (leftTemp > boundingMaxLeft) ? leftTemp : boundingMaxLeft;
			boundingMinRight = (rightTemp < boundingMinRight) ? rightTemp : boundingMinRight;
			boundingMaxRight = (rightTemp > boundingMaxRight) ? rightTemp : boundingMaxRight;
		}
		LCD.clear();
		leftMotor.stop();
		rightMotor.stop();
		LCD.drawString("Levi:  ["+boundingMinLeft+", "+boundingMaxLeft+"]",0,0);
		LCD.drawString("Desni: ["+boundingMinRight+", "+boundingMaxRight+"]",0,1);
		Delay.msDelay(1000);
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
		//readingsArray[arrayCounter % readingsArray.length] = rightReading + leftReading;
		readingsArray[arrayCounter % readingsArray.length] = (rightReading > leftReading) ? rightReading : leftReading;
		for (int reading : readingsArray) {
			total += reading;
		}
		if (total < 50) { // this number needs tweaking
			lineEnd = true;
		}
		return rightReading - leftReading;
	}
	
	public void followLine() {
		int count = 0, parameter = 100, vsota = 0, parameter2 = 1000;
		int[] arr = new int[parameter];
		int read, difference;
		for(int i=0;i<parameter;i++){
			arr[i] = getSensorReadings();
			difference = (int)myPID.compute(arr[i], 0);
			steer(difference);
		}
		while(true) {
			read = getSensorReadings(); 
			arr[count%100] = read;
			difference = (int)myPID.compute(read, 0);
			if(count % 10 == 0) {
				LCD.drawInt(difference, 0, 0);
				Sound.twoBeeps();
			}
			if(sum(arr)>parameter2){
				Sound.beepSequence();
				Sound.beepSequenceUp();
			}
			steer(difference);
		}
	}
	public int sum(int[] arr){
		int temp = 0;
		for(int i=0;i<arr.length;i++){
			temp += arr[i];
		}
		return temp;
	}

}
