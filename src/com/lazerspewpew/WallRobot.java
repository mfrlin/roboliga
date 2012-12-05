package com.lazerspewpew;

import java.io.IOException;
import java.util.Queue;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class WallRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private UltrasonicSensor usFrontSensor;
	private UltrasonicSensor usRightSensor;
	private PID myPID;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallFrontDistance; /* Kdaj reagira ko zazna zid spredaj */
	protected Queue<int[]> followData = new Queue<int[]>();
	protected int expectedCounter = 0;
	protected int leftGlobalTacho = 0;
	protected int rightGlobalTacho = 0;
	protected int leftDifferenceTacho = 0;
	protected int rightDifferenceTacho = 0;
	
	private long lastSend;
	private long sendInterval = 100;
	private int powerSampleCounter = 0;
	private int leftPowerSamples = 0;
	private int rightPowerSamples = 0;
	
	
	public WallRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		//leftReguMotor = new NXTRegulatedMotor(leftMotorPort);
		//rightReguMotor = new NXTRegulatedMotor(rightMotorPort); 
		this.wantedWallDistance = (int) (23 * 1.1); // na kaki razdalji se naj pelje desno
		this.wallFrontDistance = 26; // na kaki razdalji naj zaène upoštevati sprednji senzor
		
		usFrontSensor = new UltrasonicSensor(frontSensorPort);
		usRightSensor = new UltrasonicSensor(rightSensorPort);
		usFrontSensor.continuous(); usRightSensor.continuous();
		setMaxPower(maxPower);
//		setupPID(-maxPower, maxPower);
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
		//LCD.drawInt(difference, 5, 0, 5);
		//LCD.drawInt(leftMotor.getPower(),5, 0, 6);
		//LCD.drawInt(rightMotor.getPower(),5, 0, 7);
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
	
	/*
	 * Zavrti robota v levo, dokler ne zazna vec zidu spredaj, in dokler je dovolj odmaknjena od desnega zidu.
	 * */
	private void rotateUntilNoBarrier() {
		boolean[] neki = new boolean[] {true,true};
		int count = 0;
		do { // do at least once
			steer(this.maxPower);
			leftMotor.setPower((int) (2.5 *(wallFrontDistance / wantedWallDistance) * 3 * maxPower/4));
			rightMotor.setPower(-maxPower/3);
			//neki[count] = usFrontSensor.getDistance() < wallFrontDistance;
			//count = (count + 1) % 2;
		//} while( neki[(count + 1) % 2]);
		} while( usFrontSensor.getDistance() < 0.9 * wallFrontDistance);// || usRightSensor.getDistance() < wantedWallDistance );
		
		/* Zaradi teh delajev vozi bolj naravno. */
		Delay.msDelay(50);
		steer(0);
		Delay.msDelay(100);
		
		
	}
	
	public void followWall() {
		int steerDifference = 0;
		double steerFactor = 3;
		int counter = 0;
		long timeChange, now;
				
		while (true) {			
			LCD.clear();
			LCD.drawString("following wall", 0, 0);
			int frontDistance = usFrontSensor.getDistance();
			int rightDistance = usRightSensor.getDistance();
//			 Ce se pribliza steni spredaj se obrni za ~90deg.
			if ( frontDistance < wallFrontDistance ) {
				rotateUntilNoBarrier();
			} else {	
				// Ce imas pa zid na desni, se postavi na wantedWallDistance
				steerDifference = (int)(( wantedWallDistance - rightDistance ) * steerFactor);
				steerDifference *= steerFactor; 
				steerDifference = limit(steerDifference, (int) (maxPower * 0.5) );
				steer( steerDifference );
			}
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
	
	public void follow() {
		Thread communications = new Thread(new Communicate());
		communications.start();
		LCD.clear();LCD.drawString("Press to follow", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		double tachoAdjustCoef = 1;
		double paramAdjustCoef = 0.5;
		int leftDifference = 0;
		int rightDifference = 0;
		
		while (true) {
			/*if (usRightSensor.getDistance() < wantedWallDistance) {
				break;
			}*/
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
					int paramDifference = (int)((parameters[0] - parameters[1])/paramAdjustCoef);
					//int paramDifference = 0;
					int leftPower = parameters[2]+leftDifference-paramDifference;
					leftMotor.setPower(leftPower);
					int rightPower = parameters[3]+rightDifference+paramDifference;
					rightMotor.setPower(rightPower);
					
					
					while(leftMotor.getTachoCount() < leftGlobalTacho && rightMotor.getTachoCount() < rightGlobalTacho ) {
						if (leftMotor.getTachoCount() > leftGlobalTacho) {
							leftMotor.setPower((int)(leftPower/2));
						}
						if (rightMotor.getTachoCount() > rightGlobalTacho) {
							rightMotor.setPower((int)(rightPower/2));
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
	
	private int limit(int number, int maxBound) {
		if ( number > maxBound ){
			return maxBound;
		}else if ( number < -maxBound ) {
			return -maxBound;
		} else {
			return number;	
		}
	}


	public int awaitForStartSignal() {
		int i = -999;
		try {
			i = receiveInt();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return i;
		// TODO only return from the function when you receive the start signal from line follower;
		
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

	public void driveStraightUntilWall() {
		leftMotor.setPower(50);
		rightMotor.setPower(50);
		while (true) {
			int frontDistance = usFrontSensor.getDistance();
			if (frontDistance < wallFrontDistance) {
				rotateInPlace(-100, 810);
				break;
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
	
//	public void followWall() {
//		int read, difference;
//		long now, timeChange;
////		try {
////			int startSignal = inputStream.readInt();
////		} catch (IOException e) {
////			e.printStackTrace();
////		}
//		
//		while(true) {
//			read = getSensorReadings(); 
//			difference = (int)myPID.compute(read, wantedWallDistance);
//			steer(difference);
//			// pošiljanje podatkov
//			/*now = System.currentTimeMillis();
//			timeChange = now - lastSend;
//			if (timeChange >= sendInterval) {
//				sendTachoCounts();
//				lastSend = now;
//			}*/
//		}
//	}
}
