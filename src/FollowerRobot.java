import java.io.IOException;
import java.util.Queue;

import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;


public class FollowerRobot extends Robot {
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	
	private Queue<int[]> followData = new Queue<int[]>();
	
	public FollowerRobot(MotorPort leftMotorPort, MotorPort rightMotorPort) {
		leftMotor = new NXTRegulatedMotor(leftMotorPort);
		rightMotor = new NXTRegulatedMotor(rightMotorPort);
	}
	
	public void follow() {
		Thread communications = new Thread(new Communicate());
		communications.run();
		while (true) {
			if (!followData.empty()) {
				int[] distances = followData.pop();
				leftMotor.setSpeed(distances[0]);
				rightMotor.setSpeed(distances[1]);
				leftMotor.rotate(distances[0], true);
				rightMotor.rotate(distances[1]);
				leftMotor.waitComplete();
			}
		}
		
	}
	
	public void getTachoCounts() {
		int[] temp = new int[2];
		try {
			if (inputStream.available() != 0) {
				temp[0] = inputStream.readInt();
				temp[1] = inputStream.readInt();
				followData.push(temp);
			}
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
