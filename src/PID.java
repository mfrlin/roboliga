import lejos.nxt.LCD;

public class PID {

	private long lastTime;
	private double lastOutput;
	private double errSum, lastErr;
	private double kp, ki, kd;
	private int sampleTime = 20; // milliseconds

	public PID(double kp, double ki, double kd) {
		setTunings(kp, ki, kd);

	}

	public double compute(double input, double setpoint) {
		long now = System.currentTimeMillis();
		LCD.drawInt((int)input, 5, 0, 7);
		long timeChange = now - lastTime;
		if (timeChange >= sampleTime) {
			/*Compute all the working error variables*/
			double error = setpoint - input;
			errSum += error;
			double dErr = (error - lastErr);

			/*Compute PID Output*/
			double output = kp * error + ki * errSum + kd * dErr;

			/*Remember some variables for next time*/
			lastErr = error;
			lastTime = now;
			lastOutput = output;
			return output;

		}
		return lastOutput;
	}

	public void setTunings(double kp, double ki, double kd) {
		  double sampleTimeInSec = ((double)sampleTime)/1000;
		  this.kp = kp;
		  this.ki = ki * sampleTimeInSec;
		  this.kd = kd / sampleTimeInSec;
	}

	public void SetSampleTime(int newSampleTime) {
		if (newSampleTime > 0) {
			double ratio  = (double)newSampleTime / (double)sampleTime;
			ki *= ratio;
			kd /= ratio;
			sampleTime = newSampleTime;
	   }
	}

}