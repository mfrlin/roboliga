package com.lazerspewpew;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;


public class Robot {
	protected PID myPID;
	protected DataInputStream inputStream;
	protected DataOutputStream outputStream;
	protected int maxPower;
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	
	public void setupPID(int lowerBound, int higherBound){
//		myPID = new PID(3, 0.03, 0.03, lowerBound, higherBound);
		myPID = new PID(1, 0.03, 0.03, lowerBound, higherBound);
	}
	
	/* Tries to connect to a remote device. It takes name or address of the remote device as an argument.
	 * Remote device must be paired with the brick beforehand. Returns true if connection is made, otherwise it returns false. */ 
	public boolean connectToRemote(String name) {
		try {
			RemoteDevice receiver = Bluetooth.getKnownDevice(name);
			if (receiver == null) throw new IOException("no such device");
			
			NXTConnection connection = Bluetooth.connect(receiver);
			if (connection == null) throw new IOException("Connect fail");
			
			
			inputStream = connection.openDataInputStream();
			outputStream = connection.openDataOutputStream();
			Sound.beep();
			return true;
		} catch (Exception ioe) {
			return false;
		}
	}
	
	/* Listens for connection. If connection is established returns true, otherwise it returns false. */
	public boolean actAsReceiver() {
		try {
			LCD.drawString(Bluetooth.getFriendlyName(), 0, 0);
			LCD.drawString("Waiting for connnection.", 0, 0);
			NXTConnection connection = Bluetooth.waitForConnection();
			LCD.clear();
			LCD.drawString("Open for connection.", 0, 0);
			if (connection == null) throw new IOException("Connect fail");
			
			inputStream = connection.openDataInputStream();
			outputStream = connection.openDataOutputStream();
			Sound.twoBeeps();
			return true;
		} catch (Exception ioe) {
			LCD.clear();
			return false;
		}
	}
	
	/* Tries to send an integer. */
	public boolean sendInt(int number) {
		try {
			outputStream.writeInt(number);
			return true;
		} catch (Exception ioe) {
			return false;
		}
	}
	
	/* Looks if there is any data on the input stream and return true or false */
	public boolean dataAvailable() {
		try {
			if (inputStream.available() == 0) return false;
			else return true;
		} catch (Exception ioe) {
			return false;
		}
	}
	
	public int receiveInt() throws IOException {
		return inputStream.readInt();
	}
}
