import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class StartPoint {

	public static void main(String[] args) {
		int robotPower = 20;
		Robot gluh = new Robot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S2, robotPower);
		PID myPID = new PID(2, 0.0, 0.0);
		LCD.drawString("Ready.", 0, 0);
		Button.waitForAnyPress();
		while (true) {
			int read = gluh.getSensorReadings();
			LCD.drawInt(read, 5, 0, 1);
			int steering = (int)myPID.compute(read, 0);
			LCD.drawInt(steering, 5, 0, 2);
			gluh.steer(steering);
			
		}
	}

}
