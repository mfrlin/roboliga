package com.lazerspewpew;
import java.io.IOException;
import java.util.Queue;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.util.Delay;


public class FollowerRobot extends Robot {
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	
	protected Queue<int[]> followData = new Queue<int[]>() {{
		
	}};
	
	public FollowerRobot(MotorPort leftMotorPort, MotorPort rightMotorPort) {
		leftMotor = new NXTRegulatedMotor(leftMotorPort);
		rightMotor = new NXTRegulatedMotor(rightMotorPort);
	}
	
	public void follow() {
		//Thread communications = new Thread(new Communicate());
		//communications.run();
		
		//leftMotor.setAcceleration(3000);
		//rightMotor.setAcceleration(3000);
		leftMotor.flt(true);
		rightMotor.flt(true);

		LCD.drawString("comm true", 0, 2);
		int counter = 0;
		while (true) {
			getTachoCounts();
			//if (!followData.empty()) {
			if (!followData.isEmpty()) {
				if (counter == 0) {
					Delay.msDelay(5000);
					leftMotor.setSpeed(740);
					rightMotor.setSpeed(740);
					leftMotor.forward();
					rightMotor.forward();
					Delay.msDelay(500);
					counter++;
					
					
				}
				LCD.clear();
				
				int[] parameters = (int[]) followData.pop();
				//leftMotor.setSpeed(parameters[0]*(1000/parameters[2]));
				//rightMotor.setSpeed(parameters[0]*(1000/parameters[2]));
				//leftMotor.rotate(parameters[0], true);
				//rightMotor.rotate(parameters[1]);
				//leftMotor.waitComplete();
				LCD.drawInt(parameters[0], 4, 0, 0);
				LCD.drawInt(parameters[1], 4, 0, 0);
				leftMotor.setSpeed(parameters[0]);
				rightMotor.setSpeed(parameters[1]);
				Delay.msDelay(parameters[2]);
				
			}
		}
		
	}
	
	public void getTachoCounts() {
		int[] temp = new int[3];
		try {
			if (inputStream.available() != 0) {
				temp[0] = inputStream.readInt();
				temp[1] = inputStream.readInt();
				temp[2] = inputStream.readInt();
				followData.push(temp);
			}
			LCD.drawInt(temp[0], 0, 1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private class Communicate implements Runnable {
		public void run() {
			
			while (true) {
				getTachoCounts();
				
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
}
