package com.lazerspewpew;
import java.io.IOException;

import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class LineRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private int maxPower;
	private PID myPID;
	private long lastSend;
	private long sendInterval = 1000;
	//private int historyArrayLength = 100;
	//int[] steerHistory = new int[100];
	
	
	public LineRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
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
