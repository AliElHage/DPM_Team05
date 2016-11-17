package CompetitionExecution;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Database for all information, links to servers and clients
 * @author courtneywright
 *
 */
public class Main extends Thread{

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor pulleyMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final double WHEEL_RADIUS = 2.1;
	public static final double WIDTH = 19.1;
	private static final Port usPort1 = LocalEV3.get().getPort("S2");	
	private static final Port usPort2 = LocalEV3.get().getPort("S1");
	private static final Port usPort3 = LocalEV3.get().getPort("S3");
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	public static int BTN, BSC, CTN, CSC, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	
	public static void main(String[] args) {
		/**
		 * Instantiation for Parameter Interpretation
		 */
		ParameterInterpretation parInt = new ParameterInterpretation();
		//parInt.interpret();
		
		/**
		 * US declarations
		 */
		SensorModes usSensor1 = new EV3UltrasonicSensor(usPort1);
		SampleProvider usValue1 = usSensor1.getMode("Distance");
		SensorModes usSensor2 = new EV3UltrasonicSensor(usPort2);
		SampleProvider usValue2 = usSensor2.getMode("Distance");
		SensorModes usSensor3 = new EV3UltrasonicSensor(usPort3);
		SampleProvider usValue3 = usSensor2.getMode("Distance");
		float[] usData1 = new float[usValue1.sampleSize()];
		float[] usData2 = new float[usValue1.sampleSize()];
		float[] usData3 = new float[usValue1.sampleSize()];
		
		/**
		 * Color declarations
		 */
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];
		
		/**
		 * Class instantiations
		 */
		/*USPoller leftUS = new USPoller(usValue1, usData1);
		USPoller rightUS = new USPoller(usValue2, usData2);*/
		USPoller frontUS = new USPoller(usValue3, usData3);
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		Navigation nav = new Navigation(odo);
		LCDInfo lcd = new LCDInfo(odo,frontUS);
		Localization loc = new Localization(odo, usValue1, usValue2, usData1, usData2, 
				colorValue, colorData, leftMotor, rightMotor, nav);
		
		
		
		
		Searching searching = new Searching(nav, frontUS);
		
		ClawHandler claw = new ClawHandler(clawMotor, pulleyMotor);
		
		
		/**
		 * Localize robot
		 */
		lcd.initLCD();
		loc.localize();
		loc.zeroRobot();
		odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});
		/*nav.travelTo(0, 30.48);
		
		
		
		
		//test navigation 
		nav.travelTo(20, 30.48);
		nav.travelTo(20, 40);
		nav.travelTo(20, 60);
		nav.travelTo(0, 60);
		
		
		nav.travelByPath(0, 30.48);
		nav.travelByPath(40, 50);
		nav.travelByPath(20, 0);*/
		
		
		//test searching
		/*searching.start();*/
		
		
		
		// test for claw 
		/*claw.initializeClaw();
		Delay.msDelay(3000);
		claw.lift();

		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.release();*/
		
	}
}
