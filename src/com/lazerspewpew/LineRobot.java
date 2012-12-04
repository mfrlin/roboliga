package com.lazerspewpew;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
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
	private int[] sensorMinValuesHistory = new int[10]; // Steivlo elementov naj bo SODO. Array za shranjevanje getSensorReadings().;
	private int sensorMinValuesCounter = 0; // index zadnjega shranjenega elementa v sensorDifferences
	
	private int[] steeringHistory = new int[10]; // Steivlo elementov naj bo SODO. Array za shranjevanje getSensorReadings().;
	private int steeringHistoryCounter = 0; // index zadnjega shranjenega elementa v sensorDifferences
	private double reducedPower = 1;
//	private boolean lineEnd = false;
	private long lastSend;
	private long sendInterval = 50;
	private int powerSampleCounter = 0;
	private int leftPowerSamples = 0;
	private int rightPowerSamples = 0;
	
	
	private long lastTimeLineEnd = System.currentTimeMillis(); // za omejevanje stevila podatkov v sensorMinValuesHistory
	private long lastTimeSlowDown = System.currentTimeMillis(); // za omejevanje stevila podatkov v steeringHistory
	
	public LineRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
		calibrateLightSensors(); // calibrira, in nastavi delovanje na staticno normalizacijo
		
		// Prednapolni sensorDifferences
		for (int i = 0; i < sensorMinValuesHistory.length; i++) {
			sensorMinValuesHistory[i] = 0;
		}
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
//		leftMotor.setPower(storePower);
//		rightMotor.setPower(storePower);
	}
	
	public void steer(int difference) {
		int leftPower, rightPower;
		leftPower = (difference >= 0) ? maxPower : maxPower + difference;
		rightPower = (difference >= 0) ? maxPower - difference : maxPower;
		
//		LCD.drawInt(leftPower, 0, 0);
//		LCD.drawInt(rightPower, 0, 1);
//		LCD.drawString(">", 3, 0);
//		LCD.drawString(">", 3, 1);
//		LCD.drawInt((int) ((1-reducedPower)*leftPower), 8, 0);
//		LCD.drawInt((int) ((1-reducedPower)*rightPower), 8, 1);
		// Reduce power if we are in a curve; Set in slowDownOnCurves.
		leftPower *= reducedPower;
		rightPower *= reducedPower;
//		LCD.drawInt(leftPower, 5, 0);
//		LCD.drawInt(rightPower, 5, 1);
		
		leftMotor.setPower(leftPower);
		rightMotor.setPower(rightPower);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		
		detectLineEnd(leftReading, rightReading);
		
		slowDownOnCurves(leftReading, rightReading);
		
		return rightReading - leftReading;
	}
	
	private void detectLineEnd(int leftReading, int rightReading) {
		int sampleDelta = (int)(1000 / sensorMinValuesHistory.length); // only accept data every sampleDelta ms. Will stop in 1 second.
		sampleDelta = (int)(sampleDelta / (3.0 * maxPower / 50.0)); // 2.0 is the scaling factor
		int thresh = sensorMinValuesHistory.length * 70;
		
		long now = System.currentTimeMillis();
		
		if(now - lastTimeLineEnd >= sampleDelta){
			lastTimeLineEnd = now;
			
			sensorMinValuesHistory[getSensorMinValuesCounter()] = Math.min(rightReading, leftReading);
			
			int sum = absoluteSum(sensorMinValuesHistory);
			
			if(sum > thresh){
				LCD.drawString("STOP?", 0, 3);
				//Sound.beep();
				stopAndSendStartSignal();
			}else{
				leftMotor.forward();
				rightMotor.forward();
				leftMotor.setPower(Math.max(leftMotor.getPower(), 20));
				rightMotor.setPower(Math.max(rightMotor.getPower(), 20));
			}
		}
	}
	
	public void rotateInPlace(int power, int time){
		rightMotor.setPower(0);
		leftMotor.setPower(0);	
		leftMotor.setPower(-power/2);
		rightMotor.setPower(power/2);
		Delay.msDelay(time);
		rightMotor.setPower(0);
		leftMotor.setPower(0);
	}
	
	public void stopAndSendStartSignal() {
		int time = (int)(20000 / maxPower);
		
		rotateInPlace((int)(-maxPower/2), time );  // rotate left a bit
		
		if(leftSensor.getValue() < 50 || rightSensor.getValue() < 50){
			return;
		}
		
		rotateInPlace(maxPower/2 , time * 2); // rotate right twice as much
		
		if(leftSensor.getValue() < 50 || rightSensor.getValue() < 50){
			return;
		}
		
		rotateInPlace((int)(-maxPower/2), time); //go back to starting position, so you can follow other robot
		
		if(leftSensor.getValue() < 50 || rightSensor.getValue() < 50){
			return;
		}

		leftMotor.stop();
		rightMotor.stop();
		//Sound.twoBeeps();
		sendInt(88); // FIXME: uncomment
	}

	private void slowDownOnCurves(int leftReading, int rightReading) {
		int sampleDelta = (int)(1000 / steeringHistory.length); // only accept data every sampleDelta ms.
		sampleDelta = (int)(sampleDelta / 2);
		int thresh = steeringHistory.length * 20;
		double decceleration = 0.05; // should be a fraction of 1.
		double acceleration = decceleration * 4; // should be a fraction of 1.
		
		long now = System.currentTimeMillis();
		
		if(now - lastTimeSlowDown >= sampleDelta){
			lastTimeSlowDown = now;
			
			// Store the difference into history array
			steeringHistory[getSteeringHistoryCounter()] = rightReading-leftReading;
			
			int sum = absoluteSum(steeringHistory);
			LCD.clear();
			LCD.drawInt(sum, 0, 6);
			LCD.drawInt(thresh, 0, 7);
			
			if(sum > thresh){
				LCD.drawString("SLOW", 0, 4);
//				Sound.twoBeeps();
				reducedPower -= (reducedPower - 1) * decceleration; // 0.5 mean a reduction of up to 50% in speed.
			}else{
				reducedPower += (1 - reducedPower) * acceleration;
			}
			
		}
	}

	private int getSensorMinValuesCounter() {
		sensorMinValuesCounter = sensorMinValuesCounter < sensorMinValuesHistory.length-1 ? sensorMinValuesCounter + 1 : 0;
		return sensorMinValuesCounter;
	}

	private int getSteeringHistoryCounter() {
		steeringHistoryCounter = steeringHistoryCounter < steeringHistory.length - 1 ? steeringHistoryCounter + 1 : 0;
		return steeringHistoryCounter;
	}

	private static int absoluteSum(int[] array) {
		int sum = 0;
		int len = array.length;
		len = len%2==0 ? len - 1 : len - 2; // Ce array nima sodo stevilo elementov, enega spusti.
		for(int i = len; i>0; i-=2){ // fastest loop
			sum = sum + Math.abs(array[i]) + Math.abs(array[i-1]);
		}
		return sum;
	}

	public void followLine() {
		int read, steer;
		int counter = 0;
		long timeChange, now;
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		while(true) {
			read = getSensorReadings();  /* Skaliranje vrednosti raje opravi v NormalizedLightSensor */
			steer = (int)myPID.compute(read, 0);
			steer(steer);
			now = System.currentTimeMillis();
			timeChange = now - lastSend;
			leftPowerSamples += leftMotor.getPower();
			rightPowerSamples += rightMotor.getPower();
			powerSampleCounter++;
			if (timeChange >= sendInterval) {
				sendTachoCounts(counter);
				counter++;
				
				lastSend = now;
			}
		}
	}
	
	public boolean sendTachoCounts(int counter) {
		int leftWheel = leftMotor.getTachoCount();
		leftMotor.resetTachoCount();
		int rightWheel = rightMotor.getTachoCount();
		rightMotor.resetTachoCount();
		int leftPower = leftPowerSamples/powerSampleCounter;
		int rightPower = rightPowerSamples/powerSampleCounter;

		LCD.drawInt(leftWheel, 5, 0, 0);
		LCD.drawInt(rightWheel, 5, 0, 1);
		LCD.drawInt(leftPower, 5, 0, 2);
		LCD.drawInt(rightPower, 5, 0, 3);
		try {
			outputStream.writeInt(leftWheel);
			outputStream.writeInt(rightWheel);
			outputStream.writeInt(leftPower);
			outputStream.writeInt(rightPower);
			outputStream.writeInt(counter);
			
			outputStream.flush();
			powerSampleCounter = 0;
			leftPowerSamples = 0;
			rightPowerSamples = 0;
			
			return true;
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		}
	}
	
	public boolean sendReadings(int read) {
		try {
			outputStream.writeInt(read);
			outputStream.flush();
			return true;
		} catch (IOException e) {
			e.printStackTrace();
			return false;
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
