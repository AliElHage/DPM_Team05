package CompetitionExecution;

import java.util.ArrayList;
import lejos.hardware.Sound;
import lejos.utility.Delay;


public class BlockHunter extends Thread{
	
	enum State {INIT, SEARCHING, TRAVELING, AVOIDING, DRIVING}		//define three states of robot when it is hunting
	
	final static int ACCELERATION=4000, SPEED_NORMAL=200;
	final static double TARGET_DIS=6.2, OBJECT_DIS=40, GRASP_DIS= 6.7, VISION_DIS= 25;
	final static double  DETECTION_OFFSET= 12.5;
	private Navigation nav;
	private USPoller frontUS, leftUS, rightUS;
	private ClawHandler claw; 
	private boolean scanDone, isHunting;
	private ArrayList<double[]> destinations;				//store the target coordinates after sweeping search
	private State state;
	
	/**
	 * Constructor with setting up initial state
	 * @param nav
	 * @param frontUS
	 * @param leftUS
	 * @param rightUS
	 * @param claw
	 */
	public BlockHunter(Navigation nav, USPoller frontUS, USPoller leftUS, USPoller rightUS, ClawHandler claw){
		this(nav, frontUS, leftUS, rightUS, claw, State.INIT);
	}
	
	/**
	 * Constructor without setting up initial state, so thread will start with default INIT state
	 * @param nav
	 * @param frontUS
	 * @param leftUS
	 * @param rightUS
	 * @param claw
	 * @param state
	 */
	public BlockHunter(Navigation nav, USPoller frontUS, USPoller leftUS, USPoller rightUS, ClawHandler claw, State state){
		this.nav = nav;
		this.frontUS = frontUS;
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.claw = claw;
		this.scanDone = false;
		this.isHunting = true;					// a flag to control the on or off of state machine
		this.destinations = new ArrayList<>();
		this.state = state;
	}
	
	
	public void run(){
		Avoidance avoidance =  null;
				
		while(isHunting){
			switch (this.state) {
			
			case INIT:
				Searching searching = new Searching(nav, frontUS, rightUS);  //create a searching instance
				searching.start();	//start a thread keep checking if robot has rotated 360 and drive to next spot  if so
				destinations = searching.trackingTargets();
				searching.stopSearching(); 		// stop the searching thread if targets have been found
				nav.turnToDest(destinations.get(0)[0], destinations.get(0)[1]);  // turn to the first target detected
				this.approachTo(); // approach to the object to be ready for color check
				
				if (!this.isObstacle()) {
					// if target is a styrofoam, then grasp it
					Sound.beep();
					claw.grasp(); 
					destinations.remove(0);	   //remove the first target from the collection after detection									
					state = State.SEARCHING;
				} else {
					//// if target is a wooden block, then mark it on the map and travel to next target
					nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
					Sound.beepSequence();
					destinations.remove(0);	   //remove the first target from the collection after detection									
					state = State.SEARCHING;
				}
				break;
			
			case SEARCHING:	
				if(foamsCaptured()){			//if robot has captured foam blocks 
					nav.goZoneDesignated();   	//go to zone designated 
					state = State.TRAVELING;
				}
				if(destinations.isEmpty()){
					state = State.SEARCHING;
					break;
				}
				nav.setDest(destinations.get(0)[0], destinations.get(0)[1]);
				(new Thread(nav)).start();			//create a thread to run travelTo in nav
				if(checkFront()){					//when robot encounter an object 
					nav.interruptTraveling();				
					this.approachTo(); // approach to the object to be ready for object classification
					
					if (!this.isObstacle()) {
						// if target is a styrofoam, then grasp it
						Sound.beep();
						claw.grasp(); 
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.SEARCHING;
					}else {
						//// if target is a wooden block, then mark it on the map and travel to next target
						nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
						Sound.twoBeeps();
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.SEARCHING;
						}
				}
				break;
				
			case TRAVELING:
				if(checkFront()){				//when detect something in the front
					nav.interruptTraveling();				
					this.approachTo(); // approach to the object to be ready for object classification
					if (!this.isObstacle()) {
						// if target is a styrofoam, then grasp it
						Sound.beep();
						claw.grasp(); 								
						nav.resumeTravelingByPath();	//recall TravelTo with dest set before
					}else{
						// if target is a wooden block, then avoid it 
						markBlockFront();
						avoidance = new Avoidance(nav, frontUS, rightUS); 
						avoidance.start(); 		
						state = State.AVOIDING;		//set to avoiding mode
					}	
				}else if(!nav.checkDone()){
					nav.resumeTraveling(); 
				}else if(!foamsCaptured()){		//if robot hasnt captured foam blocks yet, set up searching
					state = State.INIT;
				}else{
					claw.releaseTower(); 		// release all the foam block 
					nav.goHome();
					state = State.DRIVING;		//if robot has complete mission go to driving state  
				}
				break;
			
			case DRIVING:			//Driving state will avoid anything encouter 
				if(checkFront()){				//when detect something in the front
					nav.interruptTraveling();				
					this.approachTo(); // approach to the object to be ready for avodiacne 
					avoidance = new Avoidance(nav, frontUS, rightUS); 
					avoidance.start(); 		
					state = State.AVOIDING;		//set to avoiding mode		
				}else if(!nav.checkDone()){
					nav.resumeTraveling(); 
				}else{
					return;
				}
				
			
			case AVOIDING:
				if(avoidance.handled()){
					nav.resumeTravelingByPath();
					state = State.TRAVELING;
				}
				break;
			

			default:
				break;
			}
		}
	}
	
	
	/**
	 * First robot adjust its heading which give the shortest distance to the target,
	 * and then rive robot to have a ideal distance to object in the front to be ready for object classification  
	 */
	public void approachTo(){
		double distance, angle, currentDis;
		
		(new Thread(){
			public void run(){
				nav.turn(-30, Navigation.SACNNNG_SPEED);
				nav.turn(60, Navigation.SACNNNG_SPEED);
				scanDone = true;
			}
		}).start();					// start a thread to let robot rotate
		
		distance = frontUS.readUSDistance();			// set up the initial US distance reading and angle 
		angle = nav.odometer.getAng();
		while(!this.scanDone){
			currentDis = frontUS.readUSDistance();		//get the current US distance 
			if(currentDis < distance){
				angle = nav.odometer.getAng();			//update distance and angle if it reads a small US distance 
				distance = currentDis;
			}
		}
		this.scanDone = false;							//reset the flag
		nav.turnTo(angle, true);						//turn the detected minimum US distance and approach to it
		while(frontUS.readUSDistance() > TARGET_DIS){
			nav.moveForward();
		}
		nav.stopMoving();
	}
	
	/**
	 * Mark the block in the front in the middle of traveling 
	 */
	private void markBlockFront(){
		double currentAng = nav.odometer.getAng();		
		double blockX = nav.odometer.getX() + TARGET_DIS*Math.cos(Math.toRadians(currentAng));
		double blockY = nav.odometer.getY() + TARGET_DIS*Math.sin(Math.toRadians(currentAng));
		nav.map.markBlocked(blockX, blockY);
	}
	
	/**
	 * Check if there is any object within vision range
	 * @return boolean true if there is an object in the front
	 */
	private boolean checkFront(){		
		return frontUS.readUSDistance() < VISION_DIS;
	}
	
	/**
	 * Turn off hunting stateMachine and should create a new Hunter thread to resume 
	 */
	public void stopHunting(){
		this.isHunting = false;
	}
	
	/**
	 * Resume hunting by passing a start state
	 * @param state
	 */
	public void resumeHunting(State state){
		this.setState(state);
		new Thread(this).start();
	}
	
	
	/**
	 * Check if robot has captured 3 foam blocks yet
	 * @return boolean true if it has
	 */
	public boolean foamsCaptured(){
		return ClawHandler.getfoamsCaptured()>=Searching.TARGET_NUM;
	}
	
	/**
	 * To detect object in the front is an obstacle or not
	 * @return boolean
	 */
	public boolean isObstacle(){
		(new Thread(){
			public void run(){
				nav.turn(30, Navigation.SACNNNG_SPEED);
				nav.turn(-30, Navigation.SACNNNG_SPEED);
				scanDone = true;
			}
		}).start();					// start a thread to let robot rotate
		
		double distance, currentDis;
		
		distance = leftUS.readUSDistance();			// set up the initial US distance 
		
		while(!this.scanDone){
			currentDis = leftUS.readUSDistance();		//get the current US distance 
			System.out.println("        dis:" +(int)currentDis );
			if(currentDis < distance){
				distance = currentDis;
			}
		}
		this.scanDone = false;	
		//nav.goBackward(DETECTION_OFFSET);//let robot move backward a bit to ensure robot to detect the same spot as front
		
		return distance < TARGET_DIS+DETECTION_OFFSET; // return object is an obstacle if the reading less than target distance 
	}
	

	
	
	/**
	 * Set robot current state
	 * @param state
	 */
	public void setState(State state){
		this.state = state;
	}

	

	public boolean obstacleTesting(){
		if(frontUS.readUSDistance() < TARGET_DIS){
			System.out.println("Object Detected");
			return true;
		}
		else{
			return false;
		}
	}
}
