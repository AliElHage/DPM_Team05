package CompetitionExecution;

import java.util.ArrayList;

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
	private static final Port usPortRight = LocalEV3.get().getPort("S2");			//S2 right US
	private static final Port usPortLeft = LocalEV3.get().getPort("S1");			//S1 left US
	private static final Port usPortFront = LocalEV3.get().getPort("S3");			//S3 front US
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	public static int BTN, BSC, CTN, CSC, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	public static boolean isBuilder;
	
	public static void main(String[] args) {
		/**
		 * Instantiation for Parameter Interpretation
		 */
		ParameterInterpretation parInt = new ParameterInterpretation();
		//parInt.interpret();
		
		/**
		 * Assign team role 
		 */
		/*if(BTN==TEAM_NUM){
			isBuilder = true;
		}else{
			isBuilder = false;
		}*/
		
		/**
		 * US declarations
		 */
		SensorModes usSensorRight = new EV3UltrasonicSensor(usPortRight);
		SampleProvider usValueRight = usSensorRight.getMode("Distance");
		SensorModes usSensorLeft = new EV3UltrasonicSensor(usPortLeft);
		SampleProvider usValueLeft = usSensorLeft.getMode("Distance");
		SensorModes usSensorFront = new EV3UltrasonicSensor(usPortFront);
		SampleProvider usValueFront = usSensorFront.getMode("Distance");
		float[] usDataRight = new float[usValueRight.sampleSize()];
		float[] usDataLeft = new float[usValueLeft.sampleSize()];		
		float[] usDataFront = new float[usValueFront.sampleSize()];
		
		/**
		 * Color declarations
		 */
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];
		LightPoller lightSensor = new LightPoller(colorValue, colorData);
		
		/**
		 * Class instantiations
		 */
		USPoller rightUS = new USPoller(usValueRight, usDataRight);
		USPoller leftUS = new USPoller(usValueLeft, usDataLeft);
		USPoller frontUS = new USPoller(usValueFront, usDataFront);
		
		lightSensor.start();
		rightUS.start();
		leftUS.start();
		frontUS.start();
		
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		Navigation nav = new Navigation(odo);
		OdometryCorrection correction = new OdometryCorrection(nav, odo, lightSensor);
		LCDInfo lcd = new LCDInfo(odo, frontUS, leftUS, rightUS);
		Localization loc = new Localization(odo, rightUS, leftUS, frontUS, lightSensor, leftMotor, rightMotor, nav);
//		ClawHandler claw = new ClawHandler(clawMotor, pulleyMotor, nav);
//		Searching searching = new Searching(nav, frontUS, rightUS);
//		BlockHunter blockHunter = new BlockHunter(nav, frontUS, leftUS, rightUS, claw);
		
		
		
		/**
		 * Localize robot
		 */
		lcd.initLCD();
		loc.localize();
//		loc.zeroRobot();
		Sound.beepSequence();
		
		
	
		
//		//TEST TRAVELING
//		 /************************************************************
//		 *	consider 0,0 as localization point
//		 *In each case robot will have the following waypoint on 4*4 grid
//		 *		travels to 45,75 (Grid 1,2)
//		 *		then return to 15,15 (Grid 0,0)
//		 ************************************************************/
//
//		FieldMap map = nav.getFieldMap();
//	/*	odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});  // reset odometer if skipping localization 
//
//		//Case 1: test up/right/down/left convention
//		//START ROBOT AT BOTTOM LEFT FACING RIGHT
//		
//		//make sure always traveling in right angles
//		nav.travelByPath(new Grid(0,2));
//		Sound.beep();
//		nav.travelByPath(new Grid(1,2));
//		Sound.beep();
//		nav.travelByPath(new Grid(1,0));
//		Sound.beep();
//		nav.travelByPath(new Grid(0,0));
//*/		
//		//Case 2: test upRight/DownLeft convention
//		//START ROBOT AT BOTTOM LEFT FACING RIGHT
//		/*nav.travelByPath(new Grid(1, 2));
//		nav.travelByPath(new Grid(0, 0));
//		
//		//case 3: test upLeft/Downright convention
//		//START ROBOT AT BOTTOM RIGHT FACING RIGHT
//		odo.setPosition(new double [] {60.0, 0.0,0.0},new boolean []{true, true, true});
//		nav.travelByPath(map.getGrid(1, 2));
//		nav.travelByPath(map.getGrid(0, 0));
//		*/
//		
//		//TEST SEARCHING
//
//		/*searching.start();
//		ArrayList<double[]> targets = searching.trackingTargets();
//		searching.stopSearching();
//		for(double[] target: targets){
//			nav.turnToDest(target[0], target[1]);
//			blockHunter.approachTo();
//			if(blockHunter.isObstacle()){
//				Sound.beepSequence();
//			}else{
//				Sound.beep();
//			}
//		}*/
//		
//
//		
//		//TEST NAVIGATION
//		/*odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});
//		nav.setDest(40, 50);
//		new Thread(nav).start();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		nav.setDest(70, 20);
//		new Thread(nav).start();*/
//		
//		
//		//TEST CALCPATH
//		//case 1 Right(Lower) Path
//		map.getGrid(1,2).setBlocked();
//		map.getGrid(1, 1).setBlocked();
//		nav.travelByPath(map.getGrid(2, 2));
//		
//		//case 2 Left(Upper) Path 	
//		map.getGrid(1, 1).setBlocked();
//		nav.travelByPath(map.getGrid(2, 2));
//		
//		//TEST CLAW
//		//testing basic functions claw
//	/*	claw.open();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.close();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.open();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.putDown();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.pullUp();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.putDownToObj();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.putDownToBot();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.open();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.close();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.pullUp();*/
//		
//		
//		//testing stacking foams
//	/*	claw.grasp();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.grasp();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.grasp();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.releaseTower();*/
//		
//		//TEST FOAM POSITION FIX WITH CLAW 
///*		claw.open();
//		claw.putDown();
//		while (Button.waitForAnyPress() != Button.ID_RIGHT);
//		claw.fixAndGrasp();
//		claw.pullUp();*/
//		
//		
//		
//		//TEST OBJECT DETECTION 
///*		while(true){
//			while (Button.waitForAnyPress() != Button.ID_RIGHT);
//			blockHunter.approachTo();
//			if(blockHunter.isObstacle()){
//				Sound.beepSequence();
//			}else{
//				Sound.beep();
//			}
//		}
//		*/
//		
//		
//		//TEST CORRECTION
//		/*correction.start();
//		odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});
//		nav.travelTo(90, 0);
//		nav.travelTo(0, 0);*/
//		
//		
//		
//		
//		/*while (Button.waitForAnyPress() != Button.ID_ESCAPE);
//		System.exit(0);*/
//		
	}
}
