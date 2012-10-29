import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private LightSensor leftSensor;
	private LightSensor rightSensor;
	private int maxPower;
	private PID myPID;
	
	public Robot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new LightSensor(leftSensorPort);
		rightSensor = new LightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID();
	}
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	

	public void setupPID(){
		myPID = new PID(2, 0.0, 0.0);
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
		LCD.drawInt(difference, 5, 0, 5);
		LCD.drawInt(leftMotor.getPower(),5, 0, 6);
		LCD.drawInt(rightMotor.getPower(),5, 0, 7);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getLightValue();
		int rightReading = rightSensor.getLightValue();
		LCD.drawInt(leftReading, 5, 0, 3);
		LCD.drawInt(rightReading, 5, 0, 4);
		return rightReading - leftReading;
	}

	public void followLine() {
		while(true) {
			int read = getSensorReadings() * 2;
			LCD.drawInt(read, 5, 0, 1);
			int steer =  Math.abs(read); // (int)myPID.compute(read, 0);
			LCD.drawInt(steer, 5, 0, 2);
			int direction = Math.round(Math.signum(read));
			steer(steer *= -direction);
		}
	}

}
