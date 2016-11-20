package CompetitionExecution;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	
	private final static int TEAM_NUM = 5;
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor pulleyMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final double WHEEL_RADIUS = 2.1;
	public static final double WIDTH = 19.1;
	private static final Port usPort1 = LocalEV3.get().getPort("S2");	
	private static final Port usPort2 = LocalEV3.get().getPort("S1");			//S1 left US
	private static final Port usPort3 = LocalEV3.get().getPort("S3");			//S3 front US
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	public static int BTN, BSC, CTN, CSC, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	private static boolean isBuilder;
	
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
		SampleProvider usValue3 = usSensor3.getMode("Distance");
		float[] usData1 = new float[usValue1.sampleSize()];
		float[] usData2 = new float[usValue1.sampleSize()];		//!!!!*********
																//*********there is a bug here. usData2 use usvalue1's method check it~
		float[] usData3 = new float[usValue3.sampleSize()];
		
		/**
		 * Color declarations
		 */
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];
		
		/**
		 * Class instantiations
		 */
		USPoller leftUS = new USPoller(usValue1, usData1);
		USPoller rightUS = new USPoller(usValue2, usData2);
		USPoller frontUS = new USPoller(usValue3, usData3);
		
		frontUS.start();
		leftUS.start();
		rightUS.start();
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		Navigation nav = new Navigation(odo);
		LCDInfo lcd = new LCDInfo(odo, frontUS, leftUS, rightUS);
		Localization loc = new Localization(odo, usValue1, usValue2, usData1, usData2, colorValue, colorData, leftMotor, rightMotor, nav);
		ClawHandler claw = new ClawHandler(clawMotor, pulleyMotor);
		
		Searching searching = new Searching(nav, frontUS, rightUS);
		
		
		
		
		/**
		 * Localize robot
		 */
		/*lcd.initLCD();
		loc.localize();
		loc.zeroRobot();
		Sound.beepSequence();
		
		//TEST NAVIGATION
		FieldMap map = nav.getFieldMap();
		odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});  // reset odometer if skipping localization
		
		
		 * **********************************************************
		 *	consider 0,0 as localization point
		 *In each case robot will have the following waypoint on 4*4 grid
		 *		travels to 45,75 (Grid 1,2)
		 *		then return to 15,15 (Grid 0,0)
		 ************************************************************
		 

		//Case 1: test up/right/down/left convention
		//START ROBOT AT BOTTOM LEFT FACING RIGHT
		
		//make sure always traveling in right angles
		nav.travelByPath(new Grid(0,2));
		Sound.beep();
		nav.travelByPath(new Grid(1,2));
		Sound.beep();
		nav.travelByPath(new Grid(1,0));
		Sound.beep();
		nav.travelByPath(new Grid(0,0));
		
		//Case 2: test upRight/DownLeft convention
		//START ROBOT AT BOTTOM LEFT FACING RIGHT
		nav.travelByPath(new Grid(1, 2));
		nav.travelByPath(new Grid(0, 0));
		
		//case 3: test upLeft/Downright convention
		//START ROBOT AT BOTTOM RIGHT FACING RIGHT
		odo.setPosition(new double [] {60.0, 0.0,0.0},new boolean []{true, true, true});
		nav.travelByPath(map.getGrid(1, 2));
		nav.travelByPath(map.getGrid(0, 0));*/
		
		
		//TEST SEARCHING
		//searching.start();
		
		
		//TEST object detection
		
		
		//TEST CLAW
		/*claw.initializeClaw();
		Delay.msDelay(3000);
		claw.lift();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.release();*/
		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
	}
}
