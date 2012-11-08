import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.comm.Bluetooth;

public class StartPoint {

	public static void main(String[] args) {
		/* gluh je ime bricka, ki mu NE dela ekrancek. uporabil ga bom za sledenje crte in kot receiverja, ker ga kot initiatorja ne morem,
		 * ker ne morem dodati drugega bricka na slepo pod known devices. pedro, ti bos verjetno uporabljal drug brick za sledenje zidu.
		 * uredi tale starting point in zakomentiraj bluetooth povezovanje. */
		if (Bluetooth.getFriendlyName().equals("gluh")) {
			int robotPower = 20;
			LineRobot lineFollower = new LineRobot(MotorPort.A, MotorPort.B, SensorPort.S2, SensorPort.S3, robotPower);
			lineFollower.actAsReceiver();
			LCD.drawString("Ready.", 0, 0);
			Button.waitForAnyPress();
			LCD.clear();
			lineFollower.followLine();
		}
		
		else if (Bluetooth.getFriendlyName().equals("slep")) {
			FollowerRobot robotFollower = new FollowerRobot(MotorPort.A, MotorPort.B);
			LCD.drawString("Press button to connect.", 0, 0);
			Button.waitForAnyPress(); // pocakaj s pritiskom gumba, dokler drug robot ni odprt za connection
			LCD.clear();
			robotFollower.connectToRemote("gluh");
			robotFollower.follow();
			
		}
	}
}
