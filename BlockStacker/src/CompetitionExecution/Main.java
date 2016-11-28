package CompetitionExecution;

import java.util.ArrayList;

import CompetitionExecution.BlockHunter.State;
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
	public static StartCorner startCorner;
	
	public static void main(String[] args) {
		/**
		 * Instantiation for Parameter Interpretation
		 */
		ParameterInterpretation parInt = new ParameterInterpretation();
		//parInt.interpret();
		
	/*	Main.BTN = 5;
		Main.BSC = 2;
		Main.CTN = 6;
		Main.CSC = 1;
		Main.LRZx = 5;
		Main.LRZy = 1;
		Main.URZx = 7;
		Main.URZy = 3;
		Main.LGZx = 2;
		Main.LGZy = 2;
		Main.UGZx = 4;
		Main.UGZy = 3;
		
		*//**
		 * Assign team role and  Set up startCorner
		 *//*
		if(BTN==TEAM_NUM){
			isBuilder = true;
			startCorner = StartCorner.lookupCorner(BSC);
		}else{
			isBuilder = false;
			startCorner = StartCorner.lookupCorner(CSC);
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
		BlockHunter blockHunter = new BlockHunter(nav, frontUS, leftUS, rightUS, claw, BlockHunter.State.TRAVELING);
		
		/**
		 * Init timer 
		 */
		/*Timer.startTiming(260);
		TimeKeeper timeKeeper  = new TimeKeeper(nav, blockHunter);
		timeKeeper.start();*/

		/**
		 * Localize robot
		 */
		/*lcd.initLCD();
		loc.localize();
		loc.zeroRobot();
		odo.setPosition(new double [] {startCorner.getX(),startCorner.getY(),startCorner.getAngle()},
				new boolean []{true, true, true});
		Sound.beepSequence();*/
		
		
		/**
		 * Travel to zone designated
		 * Red zone - garbage collector
		 * Green zone - tower builder
		 */
		/*nav.goZoneDesignated();
		new Thread(blockHunter).start();
		
		Delay.msDelay(200);			//wait robot to set up
		
		
		*//**
		 * Init borderMonitor
		 *//*
		BorderMonitor borderMonitor = new BorderMonitor(nav, blockHunter);
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
		
	
		
		
		//TEST travelTo()
		/*nav.travelTo(0, 30);
		nav.travelTo(30, 30);
		nav.travelTo(60, 60);
		
		
		//TEST TRAVELING
		 /************************************************************
		 *	consider 0,0 as localization point
		 *In each case robot will have the following waypoint on 4*4 grid
		 *		travels to 45,75 (Grid 1,2)
		 *		then return to 15,15 (Grid 0,0)
		 ************************************************************/

		FieldMap map = nav.getFieldMap();
	/*	odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});  // reset odometer if skipping localization 

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
*/		
		//Case 2: test upRight/DownLeft convention
		//START ROBOT AT BOTTOM LEFT FACING RIGHT
		/*nav.travelByPath(new Grid(1, 2));
		nav.travelByPath(new Grid(0, 0));
		
		//case 3: test upLeft/Downright convention
		//START ROBOT AT BOTTOM RIGHT FACING RIGHT
		odo.setPosition(new double [] {60.0, 0.0,0.0},new boolean []{true, true, true});
		nav.travelByPath(map.getGrid(1, 2));
		nav.travelByPath(map.getGrid(0, 0));
		*/
		

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
		

		//TEST NAVIGATION
		/*map.zoneBlocked(1, 1, 2, 2);
		
		odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});
		nav.setDest(map.getGrid(2, 2));
		new Thread(nav).start();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		nav.setDest(map.getGrid(2, 0));
		new Thread(nav).start();
		*/
		
		//TEST CALCPATH
//		case 1 Right(Lower) Path
//		map.getGrid(1,2).setBlocked();
//		map.getGrid(1, 1).setBlocked();
//		nav.travelByPath(map.getGrid(2, 2));
//		
//		//case 2 Left(Upper) Path 	
//		map.getGrid(1, 1).setBlocked();
//		nav.travelByPath(map.getGrid(2, 2));
		
		//TEST CLAW
		//testing basic functions claw
	/*	claw.open();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.close();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.open();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.putDown();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.pullUp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.putDownToObj();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.putDownToBot();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.open();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.close();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.pullUp();*/
		
		
		//testing stacking foams
		/*claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.grasp();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.releaseTower();
*/

		
		//TEST FOAM POSITION FIX WITH CLAW 
/*		claw.open();
		claw.putDown();
		while (Button.waitForAnyPress() != Button.ID_RIGHT);
		claw.fixAndGrasp();
		claw.pullUp();*/
		
		
		
		//TEST OBJECT DETECTION 
		/*while(true){

			while (Button.waitForAnyPress() != Button.ID_RIGHT);
			blockHunter.approachTo();
			if(blockHunter.isObstacle()){
				Sound.beepSequence();
			}else{
				Sound.beep();
			}

		}*/
		
		//TEST CORRECTION
		/*correction.start();
		odo.setPosition(new double [] {0.0, 0.0,0.0},new boolean []{true, true, true});
		nav.travelTo(90, 0);
		nav.travelTo(0, 0);*/
		
		
		//TEST wifi instruction
		/*FieldMap map = nav.getFieldMap();
		nav.goZoneDesignated();				//robot should go to the green zone 
		nav.travelByPath(map.getGrid(0, 2));	//robot should avoid red zone 
		 */		

		
		//TEST AVOIDANCE
		
		/*TestAvoidance avoi = new TestAvoidance(nav, frontUS, rightUS);
		blockHunter.approachTo();
		avoi.start();*/
		
		
		//TEST AVOIDANCE in NAVIGATION 
		TestAvoidance avoidance = null;
		nav.setDest(150, 60);
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
		}


		
		
		/*while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);*/
		
	}
}
