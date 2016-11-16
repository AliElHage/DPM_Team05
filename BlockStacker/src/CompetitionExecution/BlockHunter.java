package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class BlockHunter extends Thread{
	
	final static int UPDATING_ANGLE=3, ACCELERATION=4000, SPEED_NORMAL=200;
	final static double TARGET_DISTANCE=6.4, VISION_RANGE=30, BOARD_EDGE=62, SAFE_DISTANCE=18.0, SAFE_ANGLE=45.0;
	private Odometer odo;
	private Navigation nav;
	private LightPoller lightSensor;
	private USPoller frontUS, leftUS, rightUS;
	private EV3LargeRegulatedMotor hookMotorL, hookMotorR;
	private boolean foamCaptured;
	
	public BlockHunter(Odometer odometer, Navigation nav, USPoller frontUS, USPoller leftUS, USPoller rightUS, LightPoller lightSensor, EV3LargeRegulatedMotor hookMotorL, EV3LargeRegulatedMotor hookMotorR){
		this.odo = odometer;
		this.nav = nav;
		this.frontUS = frontUS;
		this.lightSensor = lightSensor;
		this.hookMotorL = hookMotorL;
		this.hookMotorR = hookMotorR;
		this.foamCaptured = false;
		this.hookMotorL.setAcceleration(ACCELERATION);
		this.hookMotorR.setAcceleration(ACCELERATION);
		this.hookMotorL.setSpeed(SPEED_NORMAL);
		this.hookMotorR.setSpeed(SPEED_NORMAL);
	}
	
	
	enum State {SEARCHING, TRAVELING, AVOIDING}		//define three states of robot when it is hunting
	
	public void run(){
		State state = State.SEARCHING;
		
		while(true){
			switch (state) {
			
			case SEARCHING:
				int shortestDis;
				
				Searching searching = new Searching(nav, frontUS); 			//create a thread object for searching
				searching.start();
				while(frontUS.readUSDistance() > VISION_RANGE){}	//keep checking the US reading until robot reads something within vision
				searching.stopThread();
				this.approachTo();					//approach to the object to be ready for color check
				
				if(lightSensor.colorCheck()){
					Sound.beep();
					captureFoam();
					nav.setDest(BOARD_EDGE, BOARD_EDGE); 	// initialize the dest of robot
					nav.start();
					state = State.TRAVELING;
				}else{
					Sound.twoBeeps();
					state = State.AVOIDING;
				}
				break;
			
			case TRAVELING:
				if(!foamCaptured){			
					state = State.SEARCHING;
				}else{								// if robot has captured the foam 
					if(checkObject()){				//when detecting something in the front
						nav.interruptTraveling();	//interrupt the current traveling
						approachTo();				// approach to it 
						Sound.twoBeeps();
						state = State.AVOIDING;		//set to avoiding mode
					}else if(!nav.checkDone()){
						nav.resumeTraveling(); 
					}else{
						// when the robot has carry the foam to the goal
						nav.turn(-180);   //put the foam to the desired grid
						Sound.systemSound(true, 3);
						return;
					}
				}
				break;
			
			case AVOIDING:
				if (nav.isTraveling()){
					Avoidance avoidance = new Avoidance(nav, leftUS, rightUS);
					avoidance.start();	
					try {
						avoidance.join();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}		// avoidance joins the main thread and main thread continue after avoidance finish
					state = State.TRAVELING;
				}else{
					Avoidance avoidance = new Avoidance(nav, leftUS, rightUS);
					avoidance.start();		// when robot is searching and meets block, simply avoid it
					try {
						avoidance.join();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					state = State.SEARCHING;
				}
				break;

			default:
				break;
			}
		}
	}
	
	private void captureFoam(){
		nav.revert();	
		nav.turn(180);    //rotate 180 to get back side facing the foam
		hookMotorL.rotate(200, true);
		hookMotorR.rotate(200, false);
		this.foamCaptured = true;
	}
	
	/**
	 * Drive robot to have a ideal distance to object in the front to be ready for color check
	 */
	private void approachTo(){
		while(frontUS.readUSDistance() > TARGET_DISTANCE){
			nav.moveForward();
		}
		nav.stopMoving();
	}
	
	
	/**
	 *  Check if there is any object within vision range
	 * @return
	 */
	private boolean checkObject(){
		if(frontUS.readUSDistance() < VISION_RANGE){
			return true;
		}else{
			return false;
		}
	}
	
	

	public boolean obstacleTesting(){
		if(frontUS.readUSDistance() < TARGET_DISTANCE){
			System.out.println("Object Detected");
			return true;
		}
		else{
			return false;
		}
	}
}
