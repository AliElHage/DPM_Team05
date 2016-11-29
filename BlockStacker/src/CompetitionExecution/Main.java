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
public class Main {
	
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
	public static StartCorner startCorner;
	private boolean scanDone, isHunting;
	private ArrayList<double[]> destinations;				//store the target coordinates after sweeping search

	
	public static void main(String[] args) {
		/**
		 * Instantiation for Parameter Interpretation
		 */
		ParameterInterpretation parInt = new ParameterInterpretation();
		//parInt.interpret();
		
		Main.BTN = 2;
		Main.BSC = 2;
		Main.CTN = 5;
		Main.CSC = 3;
		Main.LRZx = 5;
		Main.LRZy = 1;
		Main.URZx = 7;
		Main.URZy = 3;
		Main.LGZx = 2;
		Main.LGZy = 2;
		Main.UGZx = 4;
		Main.UGZy = 3;
		
		/**
		 * Assign team role and  Set up startCorner
		 */
		if(BTN==TEAM_NUM){
			isBuilder = true;
			startCorner = StartCorner.lookupCorner(BSC);
		}else{
			isBuilder = false;
			startCorner = StartCorner.lookupCorner(CSC);
		}
		
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
		frontUS.start();
		rightUS.start();
		leftUS.start();
		
		
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		Navigation nav = new Navigation(odo);
//		OdometryCorrection correction = new OdometryCorrection(nav, odo, lightSensor);
		LCDInfo lcd = new LCDInfo(odo, frontUS, leftUS, rightUS);
		Localization loc = new Localization(odo,frontUS, lightSensor, leftMotor, rightMotor, nav);
		ClawHandler claw = new ClawHandler(clawMotor, pulleyMotor, nav);
		Searching searching = new Searching(nav, frontUS, rightUS);
		BlockHunter blockHunter = new BlockHunter(nav, frontUS, leftUS, rightUS, claw);
		
		/**
		 * Init timer 
		 */
	/*	Timer.startTiming(260);
		TimeKeeper timeKeeper  = new TimeKeeper(nav, blockHunter);
		timeKeeper.start();

		*//**
		 * Localize robot
		 *//*
		lcd.initLCD();
		loc.localize();
		loc.zeroRobot();
		odo.setPosition(new double [] {startCorner.getX(),startCorner.getY(),startCorner.getAngle()},
				new boolean []{true, true, true});
		Sound.beepSequence();
		
		
		*//**
		 * Travel to zone designated
		 * Red zone - garbage collector
		 * Green zone - tower builder
		 */
		/*nav.goZoneDesignated();
		new Thread(blockHunter).start();
		
		//Delay.msDelay(200);			//wait robot to set up
		
		
		/**
		 * Init borderMonitor
		 
		/*BorderMonitor borderMonitor = new BorderMonitor(nav, blockHunter);
		borderMonitor.start();*/
		
		
		
		/*********************************************************
		 * END COMPETITION EXECUTION CODE
		 ********************************************************/
		
		
		
		
		
	/*	//TEST startCorner
		startCorner = StartCorner.lookupCorner(3);*/
		
		/**
		 * Set up odometer according to the startCorner received from Wifi
		 */
	
		/*lcd.initLCD();
		lcd.initLCD();
		loc.localize();
		loc.zeroRobot();*/
		
	
		

		FieldMap map = nav.getFieldMap();
	
		

		//TEST SEARCHING		
		/*searching.start();
		ArrayList<double[]> targets = searching.trackingTargets();
		searching.stopSearching();
		for(double[] target: targets){
			nav.turnToDest(target[0], target[1]);
			blockHunter.approachTo();
			if(blockHunter.isObstacle()){
				Sound.beepSequence();
			}else{
				Sound.beep();
				claw.grasp();
			}
		}*/
		

		
		
		
		//testing stacking foams
		/*claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.releaseTower();
*/


		
		
		
		//TEST OBJECT DETECTION 
		/*while(true){

			while (Button.waitForAnyPress() != Button.ID_RIGHT);
			blockHunter.approachTo();
			if(blockHunter.isObstacle()){
				Sound.beepSequence();
			}else{
				Sound.beep();
				claw.grasp();
			}

		}*/
	

		
		//TEST AVOIDANCE
		/*TestAvoidance avoi = new TestAvoidance(nav, frontUS, rightUS);
		blockHunter.approachTo();
		avoi.start();*/
		
		//TEST AVOIDANCE in NAVIGATION 
		/*estAvoidance avoidance = null;
		nav.setDest(map.getGrid(4, 2));
		new Thread(nav).start();
		while(true){
			if(frontUS.readUSDistance() < 25){
				nav.interruptTraveling();
				blockHunter.approachTo(); // approach to the object to be ready for object classification
				if (!blockHunter.isObstacle()) {
					// if target is a styrofoam, then grasp it
					Sound.beep();
					claw.grasp(); 								
					nav.resumeTraveling();	//recall TravelTo with dest set before
				}else{
					// if target is a wooden block, then avoid it 
					avoidance = new TestAvoidance(nav, frontUS, rightUS); 
					avoidance.start(); 
					while(!avoidance.handled());
					nav.resumeTraveling();		
				}
			}
		}*/
		
		
		//TEST threads 
	/*	Timer.startTiming(150);
		TimeKeeper timeKeeper  = new TimeKeeper(nav, blockHunter);
		timeKeeper.start();
		
		Avoidance avoidance = null;
		nav.goZoneDesignated();
		while(true){
			if(frontUS.readUSDistance() < 25){
				nav.interruptTraveling();
				blockHunter.approachTo(); // approach to the object to be ready for object classification
				if (!blockHunter.isObstacle()) {
					// if target is a styrofoam, then grasp it
					Sound.beep();
					claw.grasp(); 								
					nav.resumeTraveling();	//recall TravelTo with dest set before
				}else{
					// if target is a wooden block, then avoid it 
					avoidance = new Avoidance(nav, frontUS, rightUS); 
					avoidance.start(); 
					while(!avoidance.handled());
					nav.resumeTraveling();		
				}
			}
		}*/
		
		
		
		/*nav.setDest(map.getGrid(4, 2));
		new Thread(nav).start();*/
		nav.goZoneDesignated();
		blockHunter.startHunting(BlockHunter.State.TRAVELING);
		
		
		
		
		/*while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);*/
		
	}
	
}
