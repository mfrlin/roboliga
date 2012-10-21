

public class PID {
	
	private long lastTime;
	private double errSum, lastErr;
	private double kp, ki, kd;
	private int sampleTime = 5; // milliseconds
	
	public PID(double kp, double ki, double kd) {
		setTunings(kp, ki, kd);
		
	}
	
	// druga verzija PIDa, ki ima vdelano, da 
	public double compute(double input, double setpoint) {
		long now = System.currentTimeMillis();
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
			return output;
			
		}
		/* V tistem linku so Input, Setpoint, Output pointerji zato njegova metoda compute ne rabi vraèat nièesar,
		 * ker bere in piše direktno v spremenljivke. Tukaj pa sem moral narediti, da vrne nekaj tudi, 
		 * èe še ni èas za ponoven izraèun. 
		 */
		return -1;
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
