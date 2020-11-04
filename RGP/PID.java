package robotics;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.motor.UnregulatedMotor;

public class PID extends Thread {

	UnregulatedMotor motorC = new UnregulatedMotor(MotorPort.C);
	UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.B);

	EV3LargeRegulatedMotor motor = new EV3LargeRegulatedMotor(MotorPort.A);

	EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S1);
	SampleProvider distanceMode = ultra.getDistanceMode();
	
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	float[] ultrasonicSample = new float[distanceMode.sampleSize()];

	int lightValue;
	

	public PID() {
	}
	
	public static void main(String[] args) throws InterruptedException {
		Thread pd = new Thread(new PID());
		pd.start();
	}

	public void run() {

		int offset = (9 + 47) / 2;
		
		double kp = 1.6;
		double ki = 0.00;
		double kd = 3.0;

		double error = 0.0001;
		double integral = 0;
		double derivative = 0;
		double lastError = 0;
		double correction = 0;
		
		final int speed = 30;
		
		double powerC;
		double powerB;
		
		motor.rotateTo(0, true);

		while (true) {
			 
			while(isRed()) {
				motorC.stop();
				motorB.stop();
				Delay.msDelay(50);
			}

			if (distance() < 0.12) {
				motorC.close();
				motorB.close();

				EV3LargeRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);
				EV3LargeRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.C);

				mR.rotate(-150, true);
				mL.rotate(180);
				
				motor.rotateTo(-80);
				
				mR.close();
				mL.close();

				motorC = new UnregulatedMotor(MotorPort.C);
				motorB = new UnregulatedMotor(MotorPort.B);

				while ((int) getAverage() > offset) {
					
					 if (distance() > 0.12) {
					
					 motorB.setPower((int) (30 + (30 / 1.5)));
					 motorB.forward();
					
					 motorC.setPower(30);
					 motorC.forward();
					 if((int) getAverage() <= offset)break;
					
					 }
					
					 if (distance() <= 0.12) {
					 motorB.setPower((int) (30 - (30 / 1.5)));
					 motorB.forward();
					
					 motorC.setPower(30);
					 motorC.forward();
					 if((int) getAverage() <= offset)break;
					 }
					
					 if((int) getAverage() <= offset)break;
					 getAverage();

				}
		
				motorB.stop();
				motorC.stop();
				
				motorB.close();
				motorC.close();
				
				Delay.msDelay(10);
				
				EV3LargeRegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.B);
				EV3LargeRegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.C);

				mRight.rotate(-130, true);													
				mLeft.rotate(280);
				
				motor.rotateTo(0);
			
				mRight.close();
				mLeft.close();

				Delay.msDelay(100);
				
				motorC = new UnregulatedMotor(MotorPort.C);
				motorB = new UnregulatedMotor(MotorPort.B);
				
				do {
					motorB.setPower(30);
					motorB.forward();
					motorC.setPower(30);
					motorC.forward();
				}while ((int) getAverage() > offset);
			
				error = 0.0001;
				integral = 0;
			 	derivative = 0;
				lastError = 0;
				correction = 0;
							

			} else {

				lightValue = (int) getAverage();

				error = lightValue - offset;
				integral = error + integral;
				derivative = error - lastError;

				correction = kp * error + ki * integral + kd * derivative;

				powerB = speed + correction;
				powerC = speed - correction;

				motorB.setPower(new Double(powerB).intValue());
				motorB.forward();
				motorC.setPower(new Double(powerC).intValue());
				motorC.forward();

				lastError = error;
			}

		}
	}

	float distance() {
		ultra.getDistanceMode().fetchSample(ultrasonicSample, 0);
		return ultrasonicSample[0];

	}

	public int getAverage() {
		float[] rgb = new float[3];
		int sum = 0;		
		for (int i = 0; i < 3; i++) {
			colorSensor.getRGBMode().fetchSample(rgb, 0);
			sum += (rgb[0] + rgb[1] + rgb[2]) * 100;
		}
		return sum / 3;
	}

	public boolean isRed() {
		float[] rgb = new float[3];		
		int sum = 0;
		int red = 0;
		for (int i = 0; i < 3; i++) {
			colorSensor.getRGBMode().fetchSample(rgb, 0);
			sum += (rgb[0] + rgb[1] + rgb[2]) * 100;
			red  += rgb[0]*100;
		}
		return (red*3 > sum);
	}
}


