package com.lazerspewpew;
import java.io.IOException;
import java.util.Queue;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.util.Delay;


public class FollowerRobot extends Robot {
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	
	protected Queue<int[]> followData = new Queue<int[]>();
	
	public FollowerRobot(MotorPort leftMotorPort, MotorPort rightMotorPort) {
		leftMotor = new NXTRegulatedMotor(leftMotorPort);
		rightMotor = new NXTRegulatedMotor(rightMotorPort);
	}
	
	public void test() {
		while (true) {
		leftMotor.setSpeed(740);
		rightMotor.setSpeed(740);
		leftMotor.forward();
		rightMotor.forward();
		}
		
	}
	
	public void follow() {
		Thread communications = new Thread(new Communicate());
		communications.start();
		
		leftMotor.setAcceleration(2000);
		rightMotor.setAcceleration(2000);
		boolean firstSend = true;
		while (true) {
			int[] parameters = null;
			synchronized(followData) {
				if (!followData.empty()) {
					parameters = (int[]) followData.pop(); }
				}
			
				if (parameters != null) {
					leftMotor.resetTachoCount();
					rightMotor.resetTachoCount();
					if (firstSend) {
						Delay.msDelay(1000);
						leftMotor.setSpeed(740);
						rightMotor.setSpeed(740);
						leftMotor.rotate(370, true);
						rightMotor.rotate(370);
						leftMotor.waitComplete();
						firstSend = false; ;
					}
					leftMotor.setSpeed(parameters[0]*2);
					rightMotor.setSpeed(parameters[1]*2);
					leftMotor.rotateTo(parameters[0], true);
					rightMotor.rotateTo(parameters[1], true);
					while (leftMotor.getTachoCount() < parameters[0] -3 || rightMotor.getTachoCount() < parameters[1] -3){
						continue;
					}
					
				}
		}
	}
		

	
	public void getTachoCounts() {
		int[] temp = new int[3];
		try {
			//if (inputStream.available() != 0) {
				temp[0] = inputStream.readInt();
				temp[1] = inputStream.readInt();
				synchronized(followData) { followData.push(temp); }
			//}
			LCD.drawInt(temp[0], 0, 1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private class Communicate implements Runnable {
		public void run() {
			
			while (true) {
				getTachoCounts();
				
				/*try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}*/
			}
		}
	}
}
