package com.lazerspewpew;
import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.File;

import lejos.internal.charset.CharsetDecoder;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.util.Delay;
import java.io.*;
public class LineRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private int[] sensorMeasurements = new int[100];
//	private boolean lineEnd = false;
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
	}

	private void calibrateLightSensors() {
		int boundingMaxLeft = 0;
		int boundingMinLeft = 9999;
		int boundingMaxRight = 0;
		int boundingMinRight = 9999;
		int storePower;
		

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
		storePower = leftMotor.getPower();
		if(leftMotor.getPower() != rightMotor.getPower()) System.exit(1);
		leftMotor.setPower(0);
		rightMotor.setPower(0);
		LCD.drawString("Levi:  ["+boundingMinLeft+", "+boundingMaxLeft+"]",0,0);
		LCD.drawString("Desni: ["+boundingMinRight+", "+boundingMaxRight+"]",0,1);
		leftSensor.setFixedBoundaries(boundingMinLeft, boundingMaxLeft);
		rightSensor.setFixedBoundaries(boundingMinRight, boundingMaxRight);
		LCD.drawString("Ready to go.", 0, 2);
		Button.waitForAnyPress();
		leftMotor.setPower(storePower);
		rightMotor.setPower(storePower);
	}
	
	public void steer(int difference) {
		int leftPower, rightPower;
		leftPower = (difference >= 0) ? maxPower : maxPower + difference;
		rightPower = (difference >= 0) ? maxPower - difference : maxPower;
		leftMotor.setPower(leftPower);
		rightMotor.setPower(rightPower);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
//		int arrayCounter = 0;
//		int total = 0;
		//readingsArray[arrayCounter % readingsArray.length] = rightReading + leftReading;
//		readingsArray[arrayCounter % readingsArray.length] = (rightReading > leftReading) ? rightReading : leftReading;
//		for (int reading : readingsArray) {
//			total += reading;
//		}
//		if (total < 50) { // this number needs tweaking
//			lineEnd = true;
//		}
		
//		detectLineEnd(leftReading, rightReading);
		
		LCD.clear();
		LCD.drawInt(rightReading-leftReading, 0, 0);
		
		return rightReading - leftReading;
	}
	
	private void detectLineEnd(int leftReading, int rightReading) {
		
		
//		int count = 0, parameter = 100, vsota = 0, parameter2 = 1000, arrSum, read, difference;
//		int[] arr = new int[parameter];
//		DataOutputStream dos = createDataOutputStream("izpisRazlik.dat");
//		for(int i=0;i<parameter;i++){
//			read = getSensorReadings();
//			arr[i] = read;
//			writeIntAsString(dos,read);
//			difference = (int)myPID.compute(read, 0);
//			steer(difference);
//			Delay.msDelay(5);
//		}
//		writeIntAsString(dos,111111);
//		writeIntAsString(dos,sum(arr));
//		flushAndClose(dos);
//		Sound.beepSequenceUp();
//		Sound.beepSequence();
//		read = rightReading - leftReading;
//		arr[++count%100] = read; //getSensorReadings()
//		if(count % 10 == 0) {
//			arrSum = sum(arr);
//			LCD.clear();
//			LCD.drawInt(read, 0, 0);
//			LCD.drawInt(arrSum, 0, 1);
//			if(arrSum<parameter2){
//				leftMotor.stop();
//				rightMotor.stop();
//				Delay.msDelay(10000000);
//			}
//		}
		
	}

	public void followLine() {
		int read, steer;
		while(true) {
			read = getSensorReadings();  /* Skaliranje vrednosti raje opravi v NormalizedLightSensor */
			steer = (int)myPID.compute(read, 0);
			steer(steer);
		}
	}
	
	
	private void flushAndClose(DataOutputStream dos) {
		try {
			dos.flush();
			dos.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private DataOutputStream createDataOutputStream(String name) {
		DataOutputStream dos = null;
		try {
			dos = new DataOutputStream(new FileOutputStream(new File(name)));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		return dos;
	}

	public int sum(int[] arr){
		int temp = 0;
		for(int i=0;i<arr.length;i++){
			temp += Math.abs(arr[i]);
			//temp += 5;
		}
		return temp;
	}
	public void writeIntAsString(DataOutputStream dos, int difference){
		try {
			String a = new Integer(difference).toString();
			dos.writeBytes(a+"\n");

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
