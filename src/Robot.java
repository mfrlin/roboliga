import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;

public class Robot {
	private NXTMotor leftMotor;
	private NXTMotor rightMotor;
	private NormalizedLightSensor leftSensor;
	private NormalizedLightSensor rightSensor;
	private int maxPower;
	private PID myPID;
	private int historyArrayLength = 100;
	int[] steerHistory = new int[100];
	
	
	public Robot(MotorPort leftMotorPort, MotorPort rightMotorPort, SensorPort leftSensorPort, SensorPort rightSensorPort, int maxPower) {
		leftMotor = new NXTMotor(leftMotorPort);
		rightMotor = new NXTMotor(rightMotorPort);
		leftSensor = new NormalizedLightSensor(leftSensorPort);
		rightSensor = new NormalizedLightSensor(rightSensorPort);
		setMaxPower(maxPower);
		setupPID();
	}
	
	public void setMaxPower(int power) {
		maxPower = power;
	}
	

	public void setupPID(){
		myPID = new PID(3, 0.03, 0.03, -maxPower, maxPower);
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
		//LCD.drawInt(difference, 5, 0, 5);
		//LCD.drawInt(leftMotor.getPower(),5, 0, 6);
		//LCD.drawInt(rightMotor.getPower(),5, 0, 7);
	}
	
	public int getSensorReadings() {
		int leftReading = leftSensor.getComparableValue();
		int rightReading = rightSensor.getComparableValue();
		//LCD.drawInt(leftReading, 5, 0, 3);
		//LCD.drawInt(rightReading, 5, 0, 4);
		return rightReading - leftReading;
	}

	public void followLine() {
		int steerHistoryCount = 0;
		while(true) {
			int read = getSensorReadings();  /* Skaliranje vrednosti raje opravi v NormalizedLightSensor */
			//int read = (int) (Math.random()*40-20);
			LCD.drawInt(read, 5, 0, 1);
			int steer =  (int)myPID.compute(read, 0);
			LCD.drawInt(steer, 5, 0, 2);
			//int direction = Math.round(Math.signum(read));
			steerHistory[steerHistoryCount % historyArrayLength] = steer;
			steer(steer);
		}
	}
}
