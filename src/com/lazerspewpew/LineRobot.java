package com.lazerspewpew;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Queue;
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
	private boolean lineEnd = false;
	private long lastSend;
	private long sendInterval = 100;
	private int powerSampleCounter = 0;
	private int leftPowerSamples = 0;
	private int rightPowerSamples = 0;
	
	protected Queue<int[]> followData = new Queue<int[]>();
	protected int expectedCounter = 0;
	protected int leftGlobalTacho = 0;
	protected int rightGlobalTacho = 0;
	protected int leftDifferenceTacho = 0;
	protected int rightDifferenceTacho = 0;
	
	
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
		
		lineEnd = detectLineEnd(leftReading, rightReading);
			
		
		slowDownOnCurves(leftReading, rightReading);
		
		return rightReading - leftReading;
	}
	
	private boolean detectLineEnd(int leftReading, int rightReading) {
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
				return true;
			}else{
				leftMotor.forward();
				rightMotor.forward();
				leftMotor.setPower(Math.max(leftMotor.getPower(), 20));
				rightMotor.setPower(Math.max(rightMotor.getPower(), 20));
				return false;
			}
		}
		return false;
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
		
		rotateInPlace((int)(-maxPower/4), time); //go back to starting position, so you can follow other robot
		
		if(leftSensor.getValue() < 50 || rightSensor.getValue() < 50){
			return;
		}

		leftMotor.setPower(0);
		rightMotor.setPower(0); //TODO: tweak this
		//Sound.twoBeeps();
		sendInt(-666); // FIXME: uncomment
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
			//LCD.clear();
			//LCD.drawInt(sum, 0, 6);
			//LCD.drawInt(thresh, 0, 7);
			
			if(sum > thresh){
				//LCD.drawString("SLOW", 0, 4);
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
		while(!lineEnd) {
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
	public void driveStraight(int power, int time) {
		leftMotor.setPower(power);
		rightMotor.setPower(power);
		Delay.msDelay(time);
		leftMotor.setPower(0);
		rightMotor.setPower(0);
	}
	
	public void getTachoCounts() {
		int[] temp = new int[4];
		try {
			if (inputStream.available() != 0) {
				int first = inputStream.readInt();
				if (first == -666) {
					temp[0] = first;
					temp[1] = first;
					temp[2] = first;
					temp[3] = first;
				}
				else {
					temp[0] = first;
					temp[1] = inputStream.readInt();
					temp[2] = inputStream.readInt();
					temp[3] = inputStream.readInt();
				}
				int getCounter = inputStream.readInt();
				/*if (getCounter != expectedCounter) {
					Button.waitForAnyPress();
				}
				else {
					expectedCounter++;
				}*/
				synchronized(followData) { followData.push(temp); }
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private class Communicate implements Runnable {
		public void run() {
			
			while (true) {
				getTachoCounts();
				
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	public void follow() {
		Thread communications = new Thread(new Communicate());
		communications.start();
		Delay.msDelay(5000); //Delay za start sledenja po zamenjavi
		double tachoAdjustCoef = 0.5;
		int leftDifference = 0;
		int rightDifference = 0;
		
		while (true) {
			int[] parameters = null;
			synchronized(followData) {
				if (!followData.empty()) {
					parameters = (int[]) followData.pop(); 
				}
			}
			
			if (parameters != null) {
				if (parameters[0] == -666) {
					leftMotor.setPower(0);
					rightMotor.setPower(0);
					break;
				}
				else {
					leftGlobalTacho += parameters[0];
					rightGlobalTacho += parameters[1];
					int leftPower = parameters[2]+leftDifference;
					leftMotor.setPower(leftPower);
					int rightPower = parameters[3]+rightDifference;
					rightMotor.setPower(rightPower);
					while(leftMotor.getTachoCount() < leftGlobalTacho && rightMotor.getTachoCount() < rightGlobalTacho ) {
						if (leftMotor.getTachoCount() > leftGlobalTacho) {
							leftMotor.setPower((int)(leftPower/1.5));
						}
						if (rightMotor.getTachoCount() > rightGlobalTacho) {
							rightMotor.setPower((int)(rightPower/1.5));
						}
					}
					leftDifferenceTacho = leftGlobalTacho - leftMotor.getTachoCount();
					rightDifferenceTacho = rightGlobalTacho - rightMotor.getTachoCount();
					leftDifference = (int)(leftDifferenceTacho / tachoAdjustCoef);
					rightDifference = (int)(rightDifferenceTacho / tachoAdjustCoef);
					LCD.drawInt(leftDifference,10,0,1);
					LCD.drawInt(rightDifference,10,0,3);
					//if (leftDifference < 0) leftDifference = 0;
					//if (rightDifference < 0) rightDifference = 0;
				}
			}
		}
	}

}
