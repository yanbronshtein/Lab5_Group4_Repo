package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Search {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3UltrasonicSensor usSensor;
	private Odometer odometer;
	private Navigation navigation;
	private EV3ColorSensor cSensor;
	private static SampleProvider sampleProvider;
	private static int sampleSize;
	private static int thresholdRed = 10;
	private static double redMeanR = 14.3;
	private static double redMeanG = 2.146;
	private static double redMeanB = 1.626;
	private static double blueMeanR = 2.34;
	private static double blueMeanG = 5.82;
	private static double blueMeanB = 7.4;
	private static double yellowMeanR = 37.3;
	private static double yellowMeanG = 21.7;
	private static double yellowMeanB = 3.85;
	private static double whiteMeanR = 21.9;
	private static double whiteMeanG = 21.68;
	private static double whiteMeanB = 15.56;
	private static final int FILTER_OUT = 30;
	private int filterControl;

	public Search(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, 
			EV3UltrasonicSensor usSensor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.navigation = new Navigation(leftMotor, rightMotor);
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	}
	
	public void search(double LLx, double LLy, double URx, double URy, int TB) {	
		//first point
		double angle=0;
		boolean notFinished = true;
		navigation.turnTo(0);
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();
		while(notFinished) {
			leftMotor.setSpeed(80);
			rightMotor.setSpeed(80);
			leftMotor.setAcceleration(500);
			rightMotor.setAcceleration(500);
			leftMotor.forward();
			rightMotor.backward();
			//while(leftMotor.isMoving()) {
				//double distance = getDistance();
				double dist = getDistance();
				if(dist< 60) {
					leftMotor.stop(true);
					rightMotor.stop();
					Sound.beepSequenceUp();
					angle = odometer.getT();
					findObject(dist, TB);
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					while(true) {
						if(Math.abs(LLx - odometer.getX()) < 5 && Math.abs(LLy - odometer.getY()) < 5) {
							leftMotor.stop(true);
							rightMotor.stop();
							break;
						}
					}
					navigation.turnTo(angle + 15);
					dist = getDistance();
				}
				if(Math.abs(odometer.getT() - 90) <= 8) {
					leftMotor.stop(true);
					rightMotor.stop();
					notFinished = false;
					break;
				}
			//}	
		}
//		notFinished=true;
//		navigation.travelTo(URx,LLy);
//		navigation.turnTo(0);
//		while(notFinished) {
//			//leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle-270), true);
//			//rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle-270), true);
//			leftMotor.backward();
//			rightMotor.forward();
//			while(leftMotor.isMoving()) {
//				//double distance = getDistance();
//				if(getDistance() < 86) {
//					Sound.beep();
//					double distance = getDistance();
//					leftMotor.stop(true);
//					rightMotor.stop();
//					angle=odometer.getT();
//					findObject(distance, TB);
//					navigation.travelTo(URx,LLy);
//					navigation.turnTo(angle-10);
//				}
//			}
//			if(odometer.getT() <=270) {
//				notFinished = false;
//			}
//		}
//		notFinished=true;
//		navigation.travelTo(URx,URy);
//		navigation.turnTo(180);
//		while(notFinished) {
//			//leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
//			//rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
//			leftMotor.forward();
//			rightMotor.backward();
//			while(leftMotor.isMoving()) {
//				//double distance = getDistance();
//				if(getDistance() < 86) {
//					double distance = getDistance();
//					leftMotor.stop(true);
//					rightMotor.stop();
//					angle=odometer.getT();
//					findObject(distance, TB);
//					navigation.travelTo(URx,URy);
//					navigation.turnTo(angle+10);
//				}
//			}
//			if(odometer.getT() >=270) {
//				notFinished = false;
//			}
//		}
//		notFinished=true;
//		navigation.travelTo(LLx * Lab5.TILE_SIZE,URy * Lab5.TILE_SIZE);
//		navigation.turnTo(180);
//		while(notFinished) {
//			//leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -90), true);
//			//rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -90), true);
//			leftMotor.backward();
//			rightMotor.forward();
//			while(leftMotor.isMoving()) {
//				//double distance = getDistance();
//				if(getDistance() < 86) {
//					double distance = getDistance();
//					leftMotor.stop(true);
//					rightMotor.stop();
//					angle=odometer.getT();
//					findObject(distance, TB);
//					navigation.travelTo(LLx,URy);
//					navigation.turnTo(angle-10);
//				}
//			}
//			if(odometer.getT() <= 90) {
//				notFinished = false;
//			}
//		}
	}
	
	public void findObject(double distance, int TB) {
		//leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		//rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		leftMotor.forward();
		rightMotor.forward();
		while(true) {
			if(getDistance() < 3.5) {
				leftMotor.stop(true);
				rightMotor.stop();
				break;
			}
			if(getDistance() > distance + 10) {
				leftMotor.stop(true);
				rightMotor.stop();
				return;
			}
		}
		int color = getColor();
		if(color == TB) {
			LCD.drawString("Object Detected", 0, 0);
			//LCD.drawString("Red", 0, 1);
			Sound.beep();
		}
		else {
			Sound.twoBeeps();
		}
	}
	public int getColor() {
		sampleProvider = cSensor.getRGBMode();
	    sampleSize = sampleProvider.sampleSize();
	    float []colorSample = new float[sampleSize];
		sampleProvider.fetchSample(colorSample, 0);
		if(ifRed(colorSample[0], colorSample[1], colorSample[2])) {
			return 1;
		}
		if(ifBlue(colorSample[0], colorSample[1], colorSample[2])) {
			return 2;
		}
		if(ifYellow(colorSample[0], colorSample[1], colorSample[2])) {
			return 3;
		}
		if(ifWhite(colorSample[0], colorSample[1], colorSample[2])) {
			return 4;
		}
		return 0;
	}
	public boolean ifRed(float R, float G, float B) {
		double dr = redMeanR - R*100;
		double dg = redMeanG - G*100;
		double db = redMeanB - B*100;
		double d = Math.sqrt(dr*dr + dg*dg + db*db);
		if(d < thresholdRed) {
			return true;
		}
		return false;
	}
	public boolean ifBlue(float R, float G, float B) {
		double dr = blueMeanR - R*100;
		double dg = blueMeanG - G*100;
		double db = blueMeanB - B*100;
		double d = Math.sqrt(dr*dr + dg*dg + db*db);
		if(d < thresholdRed) {
			return true;
		}
		return false;
	}
	public boolean ifYellow(float R, float G, float B) {
		double dr = yellowMeanR - R*100;
		double dg = yellowMeanG - G*100;
		double db = yellowMeanB - B*100;
		double d = Math.sqrt(dr*dr + dg*dg + db*db);
		if(d < thresholdRed) {
			return true;
		}
		return false;
	}
	public boolean ifWhite(float R, float G, float B) {
		double dr = whiteMeanR - R*100;
		double dg = whiteMeanG - G*100;
		double db = whiteMeanB - B*100;
		double d = Math.sqrt(dr*dr + dg*dg + db*db);
		if(d < thresholdRed) {
			return true;
		}
		return false;
	}
	public double getDistance() {
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance, 0);
		double distance = usDistance[0]*100;
//		List<Double> distance = new ArrayList<Double>();
//		//sampleProvider.fetchSample(usDistance, 0);
//		for(int i = 0; i < 9; i++) {
//			sampleProvider.fetchSample(usDistance, 0);
//			LCD.drawString("us Distance: " + usDistance[0], 0, 5);
//			//distance[i] = usDistance[0]*100;
//			distance.add((double)usDistance[0]*100);
//			Delay.msDelay(30);
//		}
//		Collections.sort(distance);
//		return distance.get(9/2);
		while (distance >= 60 && filterControl < FILTER_OUT) {
		      // bad value, do not set the distance var, however do increment the
		      // filter value
		      filterControl++;
		 } 
		if (distance >= 60 && filterControl >= FILTER_OUT) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
		    	return distance;
		 } else {
		      // distance went below 255: reset filter and leave
		      // distance alone.
		      filterControl = 0;
		      return distance;
		 }
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}	
}
