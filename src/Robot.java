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
		setMaxPower(maxPower);
	}
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	
	public void steer(int difference) {		
		if (difference <= 0) { // turn right or go straight
			leftMotor.setPower(maxPower);
			rightMotor.setPower(maxPower - difference);
		}
		else { // turn left
			leftMotor.setPower(maxPower + difference);
			rightMotor.setPower(maxPower);	
		}
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getLightValue();
		int rightReading = rightSensor.getLightValue();
		return leftReading - rightReading;
	}

}
