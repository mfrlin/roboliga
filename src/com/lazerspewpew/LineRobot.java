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
	private int[] sensorDifferences = new int[10]; // array za shranjevanje getSensorReadings();
	private int arraySDCounter = 0; // Steivlo elementov naj bo SODO.
//	private boolean lineEnd = false;
	//private long lastSend;
	//private long sendInterval = 500;
	
	private long lastTime = System.currentTimeMillis(); // za omejevanje stevila podatkov v sensorDifferences
	
	public LineRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
		calibrateLightSensors(); // calibrira, in nastavi delovanje na staticno normalizacijo
		
		// Prednapolni sensorDifferences
		for (int i = 0; i < sensorDifferences.length; i++) {
			sensorDifferences[i] = 999;
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
		leftMotor.setPower(leftPower);
		rightMotor.setPower(rightPower);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		
		detectLineEnd(leftReading, rightReading);
		
		return rightReading - leftReading;
	}
	
	private void detectLineEnd(int leftReading, int rightReading) {
		int sampleDelta = 100; // only accept data every sampleDelta ms.
		int thresh = (int) (0.25 * sensorDifferences.length * ( 5 + Math.max(leftSensor.getBoundingMin(), rightSensor.getBoundingMin())));
		
		long now = System.currentTimeMillis();
		
		if(now - lastTime >= sampleDelta){
			lastTime = now;
			
			int difference = rightReading - leftReading;
			sensorDifferences[getArraySDCounter()] = difference;
			
////			Sound.beep();Sound.beep();Sound.beep();Delay.msDelay(500);
			int sum = absoluteSum(sensorDifferences);
			LCD.clear();
			LCD.drawInt(sum, 0, 0);
			LCD.drawInt(thresh, 0, 1);
			
//			
			if(sum < thresh){
//				LCD.clear();
//				LCD.drawInt(sum, 0, 0);
				LCD.drawString("STOP", 0, 3);
//				leftMotor.stop();
//				rightMotor.stop();
//				Delay.msDelay(10000000);
			}
			
			
		}
		
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

	private int getArraySDCounter() {
		arraySDCounter = arraySDCounter < sensorDifferences.length-1 ? arraySDCounter + 1 : 0;
		return arraySDCounter;
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
