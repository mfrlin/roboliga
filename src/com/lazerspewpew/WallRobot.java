package com.lazerspewpew;

import java.io.IOException;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
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
	
	public WallRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort); 
		this.wantedWallDistance = (int) (22 * 1.1); // na kaki razdalji se naj pelje desno
		this.wallFrontDistance = 24; // na kaki razdalji naj zaène upoštevati sprednji senzor
		
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
		do { // do at least once
			steer(this.maxPower);
			leftMotor.setPower(3 * maxPower/4);
			rightMotor.setPower(-maxPower/2);
		} while( usFrontSensor.getDistance() < 0.7 * wallFrontDistance || usRightSensor.getDistance() < wantedWallDistance );
		
		/* Zaradi teh delajev vozi bolj naravno. */
		Delay.msDelay(300);
		steer(0);
		Delay.msDelay(300);
		
		
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
	
	private int limit(int number, int maxBound) {
		if ( number > maxBound ){
			return maxBound;
		}else if ( number < -maxBound ) {
			return -maxBound;
		} else {
			return number;	
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
