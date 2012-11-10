package com.lazerspewpew;
import java.io.IOException;

import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class ActiveRobot extends Robot {
	private Motor leftMotor;
	private Motor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	public UltrasonicSensor usFrontSensor;
	public UltrasonicSensor usRightSensor;
	private int maxPower;
	private PID myPID;
	private long lastSend;
	private long sendInterval = 1000;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallFrontDistance; /* Kdaj reagira ko zazna zid spredaj */
	
	//private int historyArrayLength = 100;
	//int[] steerHistory = new int[100];
	
	/* This constructor is used for robot with UltraSonic sensor */
	public ActiveRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower, boolean isGluh) {
		leftMotor = new Motor(leftMotorPort, 1);
		rightMotor = new Motor(rightMotorPort, 0.93); // desni motor je mocnejsi, zato ga malo upocasni, da bo peljal naravnost
		this.wantedWallDistance = (int) (20 * 1.1); // 1.1 pride od tega, da je senzor pod kotom ~13deg.
		this.wallFrontDistance = 20; // Zeljena razdalja voznje od stene v cm.
		
		usFrontSensor = new UltrasonicSensor(frontSensorPort);
		usRightSensor = new UltrasonicSensor(rightSensorPort);
		usFrontSensor.continuous(); usRightSensor.continuous();
		setMaxPower(maxPower);
		setupPID();
	}
	
	/* This constructor is user for robot with LightSensors */
	public ActiveRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new Motor(leftMotorPort, 1);
		rightMotor = new Motor(rightMotorPort, 1);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID();
	}
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	

	public void setupPID(){
		myPID = new PID(3, 0.03, 0.03, -maxPower, maxPower);
	}
	
	public void steer(int difference) {
		if (difference >= 0) { // turn right or go straight
			leftMotor.empower(maxPower);
			rightMotor.empower(maxPower - difference);
		}
		else { // turn left
			leftMotor.empower(maxPower + difference);
			rightMotor.empower(maxPower);
		}
		//LCD.drawInt(difference, 5, 0, 5);
		//LCD.drawInt(leftMotor.getPower(),5, 0, 6);
		//LCD.drawInt(rightMotor.getPower(),5, 0, 7);
	}
	
	private void stop() {
		leftMotor.empower(0);
		rightMotor.empower(0);
	}

	/*
	 * Zavrti robota v levo, dokler ne zazna vec zidu spredaj, in dokler je dovolj odmaknjena od desnega zidu.
	 * */
	private void rotateUntilNoBarrier() {
		do { // do at least once
			steer(this.maxPower);
			leftMotor.empower(3 * maxPower/4);
			rightMotor.empower(-maxPower/2);
		} while( usFrontSensor.getDistance() < 0.8 * wallFrontDistance || usRightSensor.getDistance() < wantedWallDistance );
		
		/* Zaradi teh delajev vozi bolj naravno. */
		Delay.msDelay(300);
		steer(0);
		Delay.msDelay(300);
	}

	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		//LCD.drawInt(leftReading, 5, 0, 0);
		//LCD.drawInt(rightReading, 5, 0, 1);
		return rightReading - leftReading;
	}

	public void followLine() {
		//int steerHistoryCount = 0;
		int read, steer;
		long now, timeChange;
		
		while(true) {
			read = getSensorReadings();  /* Skaliranje vrednosti raje opravi v NormalizedLightSensor */
			//int read = (int) (Math.random()*40-20);
			//LCD.drawInt(read, 5, 0, 1);
			steer = (int)myPID.compute(read, 0);
			
			//LCD.drawInt(steer, 5, 0, 2);
			//steerHistory[steerHistoryCount % historyArrayLength] = steer;
			steer(steer);
			now = System.currentTimeMillis();
			timeChange = now - lastSend;
			if (timeChange >= sendInterval) {
				sendTachoCounts();
				lastSend = now;
			}
		}
	}
	
	public void followWall() {
		int steerDifference = 0;
		double steerFactor = 3;
				
		while (true) {			
			int frontDistance = usFrontSensor.getDistance();
			int rightDistance = usRightSensor.getDistance();
			
			// Ce se pribliza steni spredaj se obrni za ~90deg.
			if ( frontDistance < wallFrontDistance ) {
				Sound.beep();
				rotateUntilNoBarrier();
			} else {	
				// Ce imas pa zid na desni, se postavi na wantedWallDistance
				steerDifference = (int)(( wantedWallDistance - rightDistance ) * steerFactor);
				steerDifference *= steerFactor; 
				steerDifference = limit(steerDifference, (int) (maxPower * 0.5) );
				steer( steerDifference );
			}
			
//			LCD.clear();
//			LCD.drawString("Front:", 0, 0);
//			LCD.drawString("Right:", 0, 1);
//			LCD.drawInt(frontDistance, 7, 0);
//			LCD.drawInt(rightDistance, 7, 1);
			
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

	public boolean sendTachoCounts() {
		int leftWheel = leftMotor.getTachoCount();
		leftMotor.resetTachoCount();
		int rightWheel = rightMotor.getTachoCount();
		rightMotor.resetTachoCount();
		try {
			outputStream.writeInt(leftWheel);
			outputStream.writeInt(rightWheel);
			outputStream.flush();
			return true;
		} catch (IOException e) {
			
			e.printStackTrace();
			return false;
		}
	}

	public void goStraight() {
		while ( true ) {
			steer(0);	
			Delay.msDelay(500);
		}
	}
}
