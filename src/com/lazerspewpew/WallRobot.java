package com.lazerspewpew;

import java.io.IOException;
import java.util.Queue;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class WallRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NXTRegulatedMotor leftReguMotor;
	private NXTRegulatedMotor rightReguMotor;
	private UltrasonicSensor usFrontSensor;
	private UltrasonicSensor usRightSensor;
	private PID myPID;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallFrontDistance; /* Kdaj reagira ko zazna zid spredaj */
	protected Queue<int[]> followData = new Queue<int[]>();
	
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
//		long now, timeChange;
				
		while (true) {			
			LCD.clear();
			
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

//			now = System.currentTimeMillis();
//			timeChange = now - lastSend;
//			if (timeChange >= sendInterval) {
//				sendTachoCounts();
//				lastSend = now;
//			}
			
//			LCD.clear();
//			LCD.drawString("Front:", 0, 0);
//			LCD.drawString("Right:", 0, 1);
//			LCD.drawInt(frontDistance, 7, 0);
//			LCD.drawInt(rightDistance, 7, 1);
//			
//			LCD.drawString("Steer:", 0, 2);
//			LCD.drawInt(steerDifference, 7, 2);
//			Delay.msDelay(500);
		}
		
	}
	public void follow() {
		Thread communications = new Thread(new Communicate());
		communications.start();
		LCD.clear();LCD.drawString("Press to follow", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		int lastLeftTacho = 0;
		int lastRightTacho = 0;
		
		while (true) {
			int[] parameters = null;
			synchronized(followData) {
				if (!followData.empty()) {
					parameters = (int[]) followData.pop(); 
				}
			}
			
			if (parameters != null) {
				leftMotor.setPower(parameters[2]);
				rightMotor.setPower(parameters[3]);
				LCD.drawInt(parameters[0], 5, 0, 0);
				LCD.drawInt(parameters[1], 5, 0, 1);
				LCD.drawInt(parameters[2], 5, 0, 2);
				LCD.drawInt(parameters[3], 5, 0, 3);
				while(true) { // leftMotor.getTachoCount() - lastLeftTacho < parameters[0] || rightMotor.getTachoCount() - lastRightTacho < parameters[1] 
					if (leftMotor.getTachoCount() - lastLeftTacho >= parameters[0]) {
						leftMotor.setPower(0);
						lastLeftTacho = leftMotor.getTachoCount();
						lastRightTacho = rightMotor.getTachoCount();
						break;
					}
					if (rightMotor.getTachoCount() - lastRightTacho >= parameters[1]) {
						rightMotor.setPower(0);
						lastLeftTacho = leftMotor.getTachoCount();
						lastRightTacho = rightMotor.getTachoCount();
						break;
					}
				}
					
			}
		}
		
		
	}
	public void followReadings() {
		Thread communications = new Thread(new Communicate());
		communications.start();
		LCD.clear();LCD.drawString("Press to follow", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		
		while (true) {
			int[] parameters = null;
			synchronized(followData) {
				if (!followData.empty()) {
					parameters = (int[]) followData.pop(); 
				}
			}
			
			if (parameters != null) {
				
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
				temp[0] = inputStream.readInt();
				temp[1] = inputStream.readInt();
				temp[2] = inputStream.readInt();
				temp[3] = inputStream.readInt();
				synchronized(followData) { followData.push(temp); }
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	public void getReadings() {
		int[] temp = new int[4];
		try {
			if (inputStream.available() != 0) {
				temp[0] = inputStream.readInt();
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
