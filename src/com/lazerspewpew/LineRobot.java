package com.lazerspewpew;
import java.io.IOException;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class LineRobot extends Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private int maxPower;
	private PID myPID;
	private long lastSend;
	private long sendInterval = 500;
	
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
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		return rightReading - leftReading;
	}
	
	public void followLine() {
		int read, steer;
		long now, timeChange;
		
		while(true) {
			read = getSensorReadings(); 
			steer = (int)myPID.compute(read, 0);
			steer(steer);
			// pošiljanje podatkov
			/*now = System.currentTimeMillis();
			timeChange = now - lastSend;
			if (timeChange >= sendInterval) {
				sendTachoCounts();
				lastSend = now;
			}*/
		}
	}

}
