/**Class to perform light sensor localization on a grid
 * with black lines
 * @author Yin Zhang
 * @author @Lily Li  */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private EV3ColorSensor cSensor;
	private static SampleProvider sampleProvider;
	private static int sampleSize;
	private static final int ROTATE_SPEED =100;
	private static final double TILE_SIZE = Lab5.TILE_SIZE;
	
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private double thetaCorrection;
	private double angle[] = new double[4];
	private double thetay, thetax;
	
	/** Threshold to be used for detecting black gridlines
	 * based on light sensor readings */
	public static double TH;
	
	/**This the LightLocalizer constructor */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	}
	
	/**This methods localizes to the designated tiles */
	public void LLocalization(int SC) {
		if(SC==0){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(-15* Math.cos(Math.toRadians(thetay)));
			odometer.setY(-15* Math.cos(Math.toRadians(thetax)));
//			thetaCorrection = 90-(angle[0]-180)+thetay/2;
//			odometer.setTheta(odometer.getT()+thetaCorrection);
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
		}
		if(SC==1){
		}
		if(SC==2){
		}
		if(SC==3){
		}
	}
//	public void lightLocalization() {
//		int counter = 0;
//		cSensor.setFloodlight(lejos.robotics.Color.RED);
//	      //get the RGB value of the sensor
//	    sampleProvider = cSensor.getRGBMode();
//	    sampleSize = sampleProvider.sampleSize();
//	    float []colorSample = new float[sampleSize];
//	    	turnTo(45);	
//	    	leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), true);     
//	    	rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), false);
//	    	leftMotor.backward();
//	    	rightMotor.forward();
//		while(true) {
//			sampleProvider.fetchSample(colorSample, 0);
//			if(BlackLine(colorSample[0], colorSample[1], colorSample[2])) {
//				angle[counter] = odometer.getT();
//				Sound.beep();
//				counter++;
//				if(counter == 4) {
//					break;
//				}
//			}
//		}
//	}
	
	/**
	 * This method performs uses getRedMode to perform light 
	 * localization. Orients by detecting 4 gridlines *
	 */
	public void lightLocalization() {
		int counter = 0;
	      //get the RGB value of the sensor
	    sampleProvider = cSensor.getRedMode();
	    sampleSize = sampleProvider.sampleSize();
		float[] baseSample = new float[1];// base reflection of wood, useful so that it work in different lighting conditions	
		TH = baseSample[0]-(0.55*baseSample[0]);// treshold to detect a black line

	    float []colorSample = new float[sampleSize];
	    	turnTo(45);	
	    	leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), true);     
	    	rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), false);
	    	leftMotor.backward();
	    	rightMotor.forward();
		while(true) {
			sampleProvider.fetchSample(baseSample, 0);
			if(isLineCrossed(TH) == 1) {
				angle[counter] = odometer.getT();
				Sound.beep();
				counter++;
				if(counter == 4) {
					break;
				}
			}
		}
	}
	
//	public boolean BlackLine(float R,float G, float B) {
//		  if(R<=0.002 || G<=0.002 || B<=0.002) {
//			  //false if the values are too low, happens when looking at something far
//			  return false;
//		  }
//		  else if(R<=0.19 && G<=0.11 && B<=0.085) {
//			  // true if the values are within the range of a blackline
//			  return true;
//		  }
//		  else {
//			  // false if doesn't match a blackline
//			  return false;
//		  }
//	}
	
	/**When called this method samples data until it finds a black line then return something
	 * 
	 * @param TH the threshold to use to detect if a black line is crossed or not
	 * @return 1 if line crossed
	 * @return 2 if error
	 */
	public static int isLineCrossed(double TH){

		float [] currentSample;
		while(true) {// fetch samples every 0.004 seconds
			
			currentSample = new float[1];
			sampleProvider.fetchSample(currentSample,0);

			if(currentSample[0]<=TH){// if the currentsample goes under the treshold youre on a line
				System.out.println("On a line!");
				return 1;
			}

			try {
				Thread.sleep(40);
			}catch(InterruptedException e) {
				return 2;
			}
		}
	}

	
	
	/**This method navigates the robot to specified cartesian coordinates by determining
	 * the min angle using turnTo() and then navigating to that point
	 * @param x coordinate
	 * @param y coordinate
	 * @returns */
	public void travelTo(double x, double y) {
		double t;
		this.x = x;
		this.y = y;
		
		currentX = odometer.getX();
		currentY = odometer.getY();
		dx = x - currentX;
		dy = y - currentY;
		
		distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy, 2));
		t = Math.toDegrees(Math.atan2(dx, dy));
		
		turnTo(t);
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), false);
	}
		
	
	/**This method calculates the minimum angle the robot must turn to face the desired angle and then rotates the robot
	 * @param theta */
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
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method helps the robot determine the angle wheel motors need to rotate to for robot to turn to desired angle 
	 * 
	 * @param radius
	 * @param TRACK
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
