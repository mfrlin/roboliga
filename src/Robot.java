import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;

public class Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private LightSensor leftSensor;
	private LightSensor rightSensor;
	private int maxPower;
	
	public Robot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new LightSensor(leftSensorPort);
		rightSensor = new LightSensor(rightSensorPort);
	}
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	
	public void steer(int difference) {
		if (difference >= 0) { // turn right or go straight
			leftMotor.setPower(maxPower);
			rightMotor.setPower(maxPower - difference);
		}
		else { // turn left
			leftMotor.setPower(maxPower + difference);
			rightMotor.setPower(maxPower);
			
		}
		LCD.drawInt(difference, 0, 5);
		LCD.drawInt(leftMotor.getPower(), 0, 6);
		LCD.drawInt(rightMotor.getPower(), 0, 7);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getLightValue();
		int rightReading = rightSensor.getLightValue();
		LCD.drawInt(leftReading, 5, 0, 3);
		LCD.drawInt(rightReading, 5, 0, 4);
		if (leftReading < rightReading) { // we must turn left
			return leftReading - rightReading;
		}
		else { // we must turn right
			return rightReading - leftReading;
		}
	}

}
