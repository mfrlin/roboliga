import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class StartPoint {

	public static void main(String[] args) {
		int robotPower = 60;
		Robot gluh = new Robot(MotorPort.A, MotorPort.B, SensorPort.S1, SensorPort.S2, robotPower);
		LCD.drawString("Ready.", 0, 0);
		Button.waitForAnyPress();
		gluh.followLine();
	}
}
