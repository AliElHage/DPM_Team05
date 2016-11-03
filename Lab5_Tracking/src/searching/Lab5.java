package searching;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor hookMotorL = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor hookMotorR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final Port usPort = LocalEV3.get().getPort("S1");		
	private static final Port colorPort = LocalEV3.get().getPort("S2");		

	
	public static void main(String[] args) {
		
		//Setup ultrasonic sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		@SuppressWarnings("resource")							    	// Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];				
		UltrasonicPoller usPoller = new UltrasonicPoller(usValue, usData);		
		
		//Setup color sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("RGB");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
				
		// set up the odometer and display
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		Navigation nav = new Navigation(odo);
		//LCDInfo lcd = new LCDInfo(odo);
		
		//set up lightDetector and blockhunter
		final LightDetector lightDetector = new LightDetector(colorValue, colorData);
		final BlockHunter blockHunter = new BlockHunter(odo, nav, usPoller, lightDetector, hookMotorL, hookMotorR);
		
		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Block | Block  ", 0, 2);
			t.drawString("  Test | Hunting   ", 0, 3);
			t.drawString("       | ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		lightDetector.start(); 			//	start the thread to keep checking color and us reading 
		usPoller.start();
		
		if (buttonChoice == Button.ID_LEFT) {
			t.clear();
			try { Thread.sleep(1500);; } catch(Exception e){}		// wait sensors to set up
			while (true){
				while (Button.waitForAnyPress() != Button.ID_ENTER);		
				if(blockHunter.obstacleTesting()){
					lightDetector.colorTesting();
				}
			}
				
		} else {
			LCDInfo lcd = new LCDInfo(odo, usPoller, lightDetector);		//updating LCD info
			
			// perform the ultrasonic localization
			float[] usData2 = new float[usValue.sampleSize()];			//create a new buffer to avoid conflicts  
			USLocalizer usl = new USLocalizer(odo, usValue, usData2, nav);
			usl.doLocalization();
			while (Button.waitForAnyPress() != Button.ID_RIGHT);
			//odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});  //Test only*****************************************************
			blockHunter.start();
			
		}
		
	
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
