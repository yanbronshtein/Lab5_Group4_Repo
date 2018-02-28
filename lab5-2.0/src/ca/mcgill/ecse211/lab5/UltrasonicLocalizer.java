package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
/**
 * This class implements the ultrasonic sensor localizer 
 * @author Lily Li, Rene Gagnon
 *
 */
public class UltrasonicLocalizer {
	public enum LocalizationType{fallingEdge, risingEdge};
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3UltrasonicSensor usSensor;
	private Odometer odometer;
	private LocalizationType type;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED =100;
	private static final int DETECT_DISTANCE = 45;
	private double distance[];
	
	/**
	 * constructor for UltrasonicLocalizer
	 * @param leftMotor
	 * @param rightMotor
	 * @param type
	 * @param odometer
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, LocalizationType type,
			Odometer odometer, EV3UltrasonicSensor usSensor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.type = type;
		this.odometer = odometer;
		this.usSensor = usSensor;
		
	}
	/**
	 * this method implements the ultrasonic localizer
	 */
	public void UltrasonicLocalization() {
		double alpha, beta;
		//rising edge
		if(type == LocalizationType.risingEdge) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					break;
				}
			}
			//rotate to the left until it detects a rising edge
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					alpha = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Delay.msDelay(1000);
			sleep(1000);
			//keep rotating to the left until it sees a wall
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					beta = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			if(alpha < beta) {
				odometer.setTheta((45-(alpha+beta)/2) + odometer.getT());
			}
			else {
				odometer.setTheta((225-(alpha+beta)/2) + odometer.getT());
			}
			turnTo(0);			
		}
		//falling edge
		else{
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			//rotate to the left until it detects a falling edge
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					break;
				}
			}
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					alpha = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			Delay.msDelay(1000);
			sleep(1000);
			//keep rotating to the left until it sees a wall
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					beta = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			if(alpha < beta) {
				odometer.setTheta((45-(alpha+beta)/2) + odometer.getT());
			}
			else {
				odometer.setTheta((225-(alpha+beta)/2) + odometer.getT());
			}
			turnTo(0);			
		}
		
	}
	/**
	 * This method causes the robot to turn to the absolute heading theta
	 * calculate the angle based on the current heading
	 * @param theta
	 */
	public void turnTo(double theta) {
		double turnAngle;
		turnAngle = theta - odometer.getT();
		//calculate the minimal angle
		if(turnAngle < -180) {
			turnAngle = turnAngle + 360;
		}
		else if(turnAngle > 180) {
			turnAngle = turnAngle - 360;
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turnAngle), false);
	}
	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * this method fetches five samples from the ultrasonic sensor and returns the median
	 * @return distance
	 */
	public double getDistance() {
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		List<Double> distance = new ArrayList<Double>();
		//sampleProvider.fetchSample(usDistance, 0);
		for(int i = 0; i < 5; i++) {
			sampleProvider.fetchSample(usDistance, 0);
			LCD.drawString("us Distance: " + usDistance[0], 0, 5);
			//distance[i] = usDistance[0]*100;
			distance.add((double)usDistance[0]*100);
			Delay.msDelay(30);
		}
		Collections.sort(distance);
		return distance.get(5/2);
	}
	/**
	 * this method makes the program to pause for a certain amount of time
	 * @param time
	 */
	public static void sleep(int time) {
		long a = System.currentTimeMillis();
		long b = System.currentTimeMillis();
		while( (b-a) <= time) {
			b = System.currentTimeMillis();
		}
	}
}
