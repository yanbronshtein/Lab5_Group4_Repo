package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class implements the light sensor localizer,and then makes the robot to 
 * localize to four corners
 * @author Lily Li, Rene Gagnon, Yin Zhang
 *
 */
public class LightLocalizer {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private EV3ColorSensor cSensor;
	private static SampleProvider sampleProvider;
	private static int sampleSize;
	private static final int ROTATE_SPEED =100;
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private double angle[] = new double[4];
	private double thetay, thetax;
	private double thetaCorrection = 5.5;
	/**
	 * constructor for lightlocalizer
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	}
	/**
	 * localize based on the corner
	 * @param SC
	 */
	public void LLocalization(int SC) {
		if(SC==0){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(-15* Math.cos(Math.toRadians(thetay)));
			odometer.setY(-15* Math.cos(Math.toRadians(thetax)));
			odometer.setTheta(odometer.getT()+thetaCorrection);
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(30.48, 30.48, 0);
		}
		if(SC==1){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(-15* Math.cos(Math.toRadians(thetay)));
			odometer.setY(-15* Math.cos(Math.toRadians(thetax)));
			odometer.setTheta(odometer.getT()+thetaCorrection);
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(7*30.48, 30.48, 270);
		}
		if(SC==2){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(-15* Math.cos(Math.toRadians(thetay)));
			odometer.setY(-15* Math.cos(Math.toRadians(thetax)));
			odometer.setTheta(odometer.getT()+thetaCorrection);
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(7*30.48, 7*30.48, 180);
		}
		if(SC==3){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(-15* Math.cos(Math.toRadians(thetay)));
			odometer.setY(-15* Math.cos(Math.toRadians(thetax)));
			odometer.setTheta(odometer.getT()+thetaCorrection);
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(30.48, 7*30.48, 90);
		}
	}
	/**
	 * This method implements the light sensor localizer, corrects the x and y odometer
	 * reading based on the blacklines detected on the floor
	 */
	public void lightLocalization() {
		int counter = 0;
		cSensor.setFloodlight(lejos.robotics.Color.RED);
	      //get the RGB value of the sensor
	    sampleProvider = cSensor.getRGBMode();
	    sampleSize = sampleProvider.sampleSize();
	    float []colorSample = new float[sampleSize];
	    	turnTo(45);	
	    	leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), true);     
	    	rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 5 ), false);
	    	leftMotor.backward();
	    	rightMotor.forward();
		while(true) {
			sampleProvider.fetchSample(colorSample, 0);
			if(BlackLine(colorSample[0], colorSample[1], colorSample[2])) {
				angle[counter] = odometer.getT();
				Sound.beep();
				counter++;
				if(counter == 4) {
					break;
				}
			}
		}
	}
	/**
	 * this method determines if the color sensor sees a black line based on the RGB reading
	 * @param R
	 * @param G
	 * @param B
	 * @return boolean
	 */
	public boolean BlackLine(float R,float G, float B) {
		  if(R<=0.002 || G<=0.002 || B<=0.002) {
			  //false if the values are too low, happens when looking at something far
			  return false;
		  }
		  else if(R<=0.19 && G<=0.11 && B<=0.085) {
			  // true if the values are within the range of a blackline
			  return true;
		  }
		  else {
			  // false if doesn't match a blackline
			  return false;
		  }
	}
	/**
	 * This method causes the robot to move to the coordinate specified by the parameters
	 * @param x
	 * @param y
	 * @throws OdometerExceptions 
	 */
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
}
