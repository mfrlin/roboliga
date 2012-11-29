package com.lazerspewpew;

import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Delay;

public class WallRobot extends Robot {
	private UltrasonicSensor usFrontSensor;
	private UltrasonicSensor usRightSensor;
	private PID myPID;
	private int wantedWallDistance; /* Zeljena povprecna razdalja od zidu */ 
	private int wallFrontDistance; /* Kdaj reagira ko zazna zid spredaj */
	
	public WallRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort frontSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort); 
		this.wantedWallDistance = 15; // na kaki razdalji se naj pelje desno
		this.wallFrontDistance = 20; // na kaki razdalji naj zaène upoštevati sprednji senzor
		
		usFrontSensor = new UltrasonicSensor(frontSensorPort);
		usRightSensor = new UltrasonicSensor(rightSensorPort);
		usFrontSensor.continuous(); usRightSensor.continuous();
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
	}
	public void follow(){
		int lastSequenceNumber = -1;
		int currentSequencelNumber = 0, timeChange = 0, read = 0, steer;
		while(true){
			
			try {
				currentSequencelNumber = inputStream.readInt();
				timeChange = inputStream.readInt();
				read = inputStream.readInt();
			} catch (IOException e) {
				e.printStackTrace();
			}
			Sound.twoBeeps();
			LCD.drawInt(currentSequencelNumber, 0, 0);
			LCD.drawInt(lastSequenceNumber, 0,1);
			LCD.drawInt(read, 0,2);
			if(currentSequencelNumber != (lastSequenceNumber + 1)){
				Delay.msDelay(2000);
				Sound.twoBeeps();
				Sound.twoBeeps();
				Sound.twoBeeps();
				LCD.clear();
				LCD.drawString("Package loss, try again",0,0);
				LCD.drawString("Press to exit",0,1);
				Button.waitForAnyPress();
			}
			steer = (int)myPID.compute(read, 0);
			steer(steer);
		}
	}
}
