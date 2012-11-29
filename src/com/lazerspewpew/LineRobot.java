package com.lazerspewpew;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class LineRobot extends Robot {
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	//private long lastSend;
	//private long sendInterval = 500;
	
	public LineRobot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID(-maxPower, maxPower);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getValue();
		int rightReading = rightSensor.getValue();
		return rightReading - leftReading;
	}
	
	public void followLine() {
		DataOutputStream dos = createDos();                //ustvari dataOutputStream
		int read, steer, timeChange;
		long now, lastSend = System.currentTimeMillis();
		int iterCounter = 1;
		while(true) {
			read = getSensorReadings(); 
			writeReadToFile(dos, read);
			iterCounter = iterCounter + 1; 
			// pošiljanje podatkov
			if((iterCounter % 100) == 0) {//pošilja vsako pol sekunde, i.e. 100*5ms = pol sekunde, hardcodano
				now = System.currentTimeMillis();
				timeChange = (int) (now - lastSend);
				lastSend = (int) now;
				sendData( (iterCounter / 100) - 1, timeChange, read);
			}
			steer = (int)myPID.compute(read, 0);
			steer(steer);
		}
	}

	private DataOutputStream createDos() {

		File dat = new File("dat.dat");
		FileOutputStream fos = null;
		DataOutputStream dos = new DataOutputStream(fos);
		try {
			fos = new FileOutputStream(dat);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			LCD.drawString("Napaka pri ustvarjanju streama", 0, 0);
		}
		return dos;
	}
	private void writeReadToFile(DataOutputStream dos, int read){
		try {
			dos.writeInt(read);
		} catch (IOException e) {
			e.printStackTrace();
			LCD.drawString("Pisanje ni uspelo", 0, 0);
		}
	}

}
