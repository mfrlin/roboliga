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
		//DataOutputStream dos = createDos();

		int read, steer, timeChange, sequenceNumber;
		long now, lastSend = System.currentTimeMillis();
		int iterCounter = 1;
		int frekvencaPosiljanja = 1000;
		//int[] arr = new int[frekvencaPosiljanja];
		while(true) {
			read = getSensorReadings();
			iterCounter++;
			// pošiljanje podatkov
			if((iterCounter % frekvencaPosiljanja) == 0) {//pošilja vsako pol sekunde, i.e. 100*5ms = pol sekunde, hardcodano
				
				sequenceNumber = (iterCounter / frekvencaPosiljanja) - 1;
				now = System.currentTimeMillis();
				timeChange = (int) (now - lastSend);
				lastSend = (int) now;
				sendData(sequenceNumber, timeChange, read);
				/*
				LCD.drawInt(333,0,0);
				LCD.drawInt(444,0,1);
				LCD.drawInt(555,0,2);
				sendData(333, 444, 555);
				
				LCD.drawInt((iterCounter / 100) - 1,0,0);
				LCD.drawInt(timeChange,0,1);
				LCD.drawInt(read,0,2);
				*/
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
			LCD.drawString("Napaka pri ustvarjanju streama", 0, 0);
		}
		return dos;
	}
	private void writeReadToFile(DataOutputStream dos, int read){
		try {
			Sound.beepSequence();
			if(dos == null) Sound.buzz();
			dos.writeInt(555);
			Sound.beepSequenceUp();
		} catch (IOException e) {
			Sound.beepSequence();
			LCD.drawString("Pisanje ni uspelo", 0, 0);
		}
	}

}
