/**
 * Lab5.java
 * 
 */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.UltrasonicLocalizer.LocalizationType;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final EV3UltrasonicSensor  usSensor= new EV3UltrasonicSensor
			(LocalEV3.get().getPort("S1"));
	//private static final Port usPort = LocalEV3.get().getPort("S4");
	
	/**Tested wheel radius cm */
	public static final double WHEEL_RAD = 2.16;
	
	/**Width of robot cm */
	public static final double TRACK = 12.2;
	
	/**right wheel radius cm */
	public static final double leftRadius = 2.2;
	/**right wheel radius cm */
	public static final double rightRadius = 2.2;
	
	/**dimension of tile cm */
	public static final double TILE_SIZE = 30.48;
	
	/**Speed in deg/s of wheel motors */
	public static final int ROTATE_SPEED = 100;

	
	/** Bottom Left x coordinate of search area */
	public static int LLx = 2;
	
	/** Bottom Left x coordinate of search area */
	public static int LLy = 2;
	
	/** Upper Right x coordinate of search area */
	public static int URx = 6;
	/** Upper Right y coordinate of search area */
	public static int URy = 6;
	
	/**Color of target block 
	 * red-1, blue-2, yellow-3, white-4  */
	public static int TB = 4;
	
	/**Starting corner. 0 is bottom left corner */
	public static int SC = 0;

	
	/**This is the main method
	 * @throws OdometerExceptions */
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		Navigation navigation = new Navigation(leftMotor, rightMotor);
		UltrasonicLocalizer ultrasoniclocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, LocalizationType.fallingEdge,
				odometer, usSensor);
		Search search = new Search(leftMotor, rightMotor, odometer, usSensor);
		LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor,odometer);
		//OdometryCorrection odometryCorrection = new OdometryCorrection(); 
		Display odometryDisplay = new Display(lcd); // No need to change
		//Avoid avoid = new Avoid(leftMotor, rightMotor);
		do{
			lcd.clear();
 
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString(" Iden- | The    ", 0, 1);
			lcd.drawString(" fy    | whole  ", 0, 2);
			lcd.drawString("       | lab ", 0, 3);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			lcd.clear();
			while(true) {
				int color = search.getColor();
				//red
				if(color == 1) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Red", 0, 1);
				}
				//blue
				if(color == 2) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Blue", 0, 1);
				}
				//yellow
				if(color == 3) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Yellow", 0, 1);
				}
				//white
				if(color == 4) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("White", 0, 1);
				}
				if(Button.getButtons() == Button.ID_ESCAPE) {
					System.exit(0);
				}
			}

		} else {
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("Press a button", 0, 0);
				lcd.drawString(" to start        ", 0, 1);
				lcd.drawString("localization  ", 0, 2);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			ultrasoniclocalizer.UltrasonicLocalization();
			lightLocalizer.LLocalization(SC);
			do {
				buttonChoice = Button.waitForAnyPress();
			} while (buttonChoice != Button.ID_LEFT);
			odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
			navigation.travelTo(LLx*TILE_SIZE,LLy*TILE_SIZE);
			Sound.beep();
			search.search(LLx*TILE_SIZE, LLy*TILE_SIZE, URx*TILE_SIZE, URy*TILE_SIZE, TB);
			//navigation.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);


			
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
