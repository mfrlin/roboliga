import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class StartPoint {

	public static void main(String[] args) {
		int robotPower = 60;
		Robot gluh = new Robot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S2, robotPower);
		PID myPID = new PID(3, 0.05, 0.25, 0, 2*robotPower);
		LCD.drawString("Ready.", 0, 0);
		Button.waitForAnyPress();
		while (true) {
			int read = gluh.getSensorReadings();
			LCD.drawInt(read, 5, 0, 1);
			int steer = (int)myPID.compute(Math.abs(read), 0);
			LCD.drawInt(steer, 5, 0, 2);
			if (read < 0) gluh.steer(-steer);
			else gluh.steer(steer);
			
		}
	}

}
