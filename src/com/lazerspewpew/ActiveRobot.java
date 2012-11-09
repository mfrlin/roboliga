package com.lazerspewpew;
import java.io.IOException;

import javax.microedition.lcdui.Font;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class ActiveRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private UltrasonicSensor usFrontSensor;
	private UltrasonicSensor usRightSensor;
	private int maxPower;
	private PID myPID;
	private long lastSend;
	private long sendInterval = 1000;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallDistancePathWidth; /* Pri koliksni razliki od zeljene reagiramo -> doloci sirino voznega pasu */
	private boolean isGluh = false; /* Robot with UltraSonic sensors: true */
	//private int historyArrayLength = 100;
	//int[] steerHistory = new int[100];
	
	/* This constructor is used for robot with UltraSonic sensor */
	public ActiveRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower, boolean isGluh) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		this.isGluh = isGluh;
		this.wantedWallDistance = 10;
		this.wallDistancePathWidth = 4;
		
		
		usFrontSensor = new UltrasonicSensor(frontSensorPort);
		usRightSensor = new UltrasonicSensor(rightSensorPort);
		usFrontSensor.continuous(); usRightSensor.continuous();
		setMaxPower(maxPower);
		setupPID();
	}
	
	/* This constructor is user for robot with LightSensors */
	public ActiveRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
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
	
	private void stop() {
		leftMotor.setPower(0);
		rightMotor.setPower(0);
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
		while(true) {
			int read = getSensorReadings();  /* Skaliranje vrednosti raje opravi v NormalizedLightSensor */
			//int read = (int) (Math.random()*40-20);
			//LCD.drawInt(read, 5, 0, 1);
			int steer =  (int)myPID.compute(read, 0);
			//LCD.drawInt(steer, 5, 0, 2);
			//int direction = Math.round(Math.signum(read));
			//steerHistory[steerHistoryCount % historyArrayLength] = steer;
			steer(steer);
			long now = System.currentTimeMillis();
			long timeChange = now - lastSend;
			if (timeChange >= sendInterval) {
				sendTachoCounts();
				lastSend = now;
			}
		}
	}
	
	public void followWall() {
		
		int leftPathBoundary =(int)( wantedWallDistance + wallDistancePathWidth * 0.5 );
		int rightPathBoundary = (int)( wantedWallDistance - wallDistancePathWidth * 0.5 );
		int steerDifference = 0;
		double steerFactor = 1.5;
		
		while (true) {
			LCD.clear();
			
			int frontDistance = usFrontSensor.getDistance();
			int rightDistance = usRightSensor.getDistance();
			
			LCD.drawString("Front:", 0, 0);
			LCD.drawString("Right:", 0, 1);
			LCD.drawInt(frontDistance, 7, 0);
			LCD.drawInt(rightDistance, 7, 1);
			
			if ( frontDistance < 20 ) {
				Sound.beep();
				 stop();
				// TODO: turn left by 90 deg.
			} else {	
				
				if ( rightDistance < rightPathBoundary ) {
					steerDifference =  (int)(( wantedWallDistance - rightDistance ) * steerFactor);
				} else if ( rightDistance > leftPathBoundary ) {
					steerDifference = (int)(( wantedWallDistance - rightDistance ) * steerFactor);
				}
				
				steerDifference *= steerFactor; 
				
				if (steerDifference > 20) {
					steerDifference = 20;
				} else if (steerDifference < -20) {
					steerDifference = -20;
				}
				
				LCD.drawString("Steer:", 0, 2);
				LCD.drawInt(steerDifference, 7, 2);
				steer( steerDifference );
				
			}
			
			
			Delay.msDelay(500);
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
}
