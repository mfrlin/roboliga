package com.lazerspewpew;


public class PID {

	private long lastTime;
	private double ITerm, lastInput;
	private double lastOutput;
	private double kp, ki, kd;
	private int sampleTime = 5; // milliseconds
	private double outMin, outMax;

	public PID(double kp, double ki, double kd, double outMin, double outMax) {
		setTunings(kp, ki, kd);
		setOutputLimits(outMin, outMax);

	}


	public double compute(double input, double setpoint) {
		long now = System.currentTimeMillis();
		long timeChange = now - lastTime;
		if (timeChange >= sampleTime) {
			/*Compute all the working error variables*/
			double error = setpoint - input;
			ITerm += (ki * error);
			if (ITerm > outMax) ITerm = outMax;
			else if (ITerm < outMin) ITerm = outMin;
			double dInput = (input - lastInput);

			/*Compute PID Output*/
			double output = kp * error + ITerm - kd * dInput;
			if (output > outMax) output = outMax;
			else if (output < outMin) output = outMin;
			/*Remember some variables for next time*/
			lastInput = input;
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

	public void setOutputLimits(double min, double max) {
		if (min > max) return; // TODO: We should probably raise exception here
		outMin = min;
		outMax = max;

		if (lastOutput > outMax) lastOutput = outMax;
		else if (lastOutput < outMin) lastOutput = outMin;

		if (ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
	}

}