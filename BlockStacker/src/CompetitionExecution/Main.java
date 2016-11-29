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
public class Main {
	
	enum State {INIT, SEARCHING, TRAVELING, AVOIDING, DRIVING}		//define three states of robot when it is hunting
	
	final static double TARGET_DIS=6.2, OBJECT_DIS=40, GRASP_DIS= 6.7, VISION_DIS= 25;
	final static double  DETECTION_OFFSET= 12.5;
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
	private static boolean goHome = false;

	
	public static void main(String[] args) {
		/**
		 * Instantiation for Parameter Interpretation
		 */
		ParameterInterpretation parInt = new ParameterInterpretation();
		parInt.interpret();
		
		/*Main.BTN = 2;
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
		Main.UGZy = 3;*/
		
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
		Timer.startTiming(275);
		TimeKeeper timeKeeper  = new TimeKeeper(nav, blockHunter);
		timeKeeper.start();

		/**
		 * Localize robot
		 */
		lcd.initLCD();
		loc.localize();
		loc.zeroRobot();
		Sound.beepSequence();
		lightSensor.stopRunning();
		
		
		//Set up odometer according to the startCorner received from Wifi
		odo.setPosition(new double [] {startCorner.getX(),startCorner.getY(),startCorner.getAngle()},
				new boolean []{true, true, true});
		
		rightUS.start();
		leftUS.start();
		
		
		//Start moving 
		Avoidance avoidance = null;
		//First drive robot to the designated zone
		nav.goZoneDesignated();			
		//Checking front while traveling 
		while(true){
			if(nav.checkDone()){
				break;
			}
			if(frontUS.readUSDistance() < VISION_DIS){
				nav.interruptTraveling();
				blockHunter.approachTo(); // approach to the object to be ready for object classification
				if (!blockHunter.isObstacle()) {
					// if target is a styrofoam, then grasp it
					claw.grasp(); 								
					nav.resumeTraveling();	//recall TravelTo with dest set before
				}else{
					// if target is a wooden block, then avoid it 
					blockHunter.markBlockFront();
					avoidance = new Avoidance(nav, frontUS, rightUS); 
					avoidance.start(); 
					while(!avoidance.handled());
					nav.resumeTraveling();		
				}
			}
		}
		
		//once robot get to the designated zone, check if it has captured enough foam, if not start searching
		while(!blockHunter.foamsCaptured()){
			if(goHome){			// if game is close to end, break the while loop
				break;
			}
			searching = new Searching(nav, frontUS, rightUS);  //create a searching instance
			searching.start();	//start a thread keep checking if robot has rotated 360 and drive to next spot  if so
			ArrayList<double[]> targets = searching.trackingTargets();
			searching.stopSearching(); 		// stop the searching thread if targets have been found			
			for(double[] target: targets){
				nav.turnToDest(target[0], target[1]);
				blockHunter.approachTo();
				if(blockHunter.isObstacle()){
					nav.map.markBlocked(target[0], target[1]);
				}else{
					claw.grasp();
				}
			}
		}
		
		if(!goHome){	// do the following only if robot has enough time
			nav.goZoneDesignated();
			//Checking front while traveling
			while(true){
				if(nav.checkDone()){
					break;
				}
				if(frontUS.readUSDistance() < VISION_DIS){
					nav.interruptTraveling();
					blockHunter.approachTo(); // approach to the object to be ready for object classification
					if (!blockHunter.isObstacle()) {
						// if target is a styrofoam, then grasp it
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
			}
			nav.turnToZoneDesignated();
			claw.releaseTower();  //release the blocks at designated zone 
		}
		
		nav.goHome();			// go back to starting corner
		//Checking front while traveling, avoid any objects
		while(true){
			if(nav.checkDone()){
				break;
			}
			if(frontUS.readUSDistance() < VISION_DIS){
				nav.interruptTraveling();
				blockHunter.approachTo(); // approach to the object to be ready for object classification
			
				// run avoidance for any object  
				avoidance = new Avoidance(nav, frontUS, rightUS); 
				avoidance.start(); 
				while(!avoidance.handled());
				nav.resumeTraveling();		
			}
		}
		nav.goIntoHome();	//go into starting corner 
		
	}
	
	public static void timeToGoHome(){
		goHome = true;
	}
	
}
