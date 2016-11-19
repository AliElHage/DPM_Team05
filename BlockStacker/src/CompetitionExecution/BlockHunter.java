package CompetitionExecution;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class BlockHunter extends Thread{
	
	final static int UPDATING_ANGLE=3, ACCELERATION=4000, SPEED_NORMAL=200;
	final static double TARGET_DISTANCE=6.4, OBJECT_DIS=25, BOARD_EDGE=62;
	private Navigation nav;
	private LightPoller lightSensor;
	private USPoller frontUS, leftUS, rightUS;
	private ClawHandler claw; 
	private boolean foamCaptured;
	private ArrayList<double[]> destinations;				//store the target coordinates after sweeping search
	
	public BlockHunter(Navigation nav, USPoller frontUS, USPoller leftUS, USPoller rightUS, ClawHandler claw){
		this.nav = nav;
		this.frontUS = frontUS;
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.claw = claw;
		this.foamCaptured = false;
		this.destinations = new ArrayList<>();

	}
	
	enum State {SEARCHING, TRAVELING, AVOIDING}		//define three states of robot when it is hunting
	
	public void run(){
		State state = State.SEARCHING;
		
		while(true){
			switch (state) {
			
			case SEARCHING:
				nav.rotateLeft();			//set robot keeping rotating to left
				Searching searching = new Searching(nav, frontUS, rightUS); 		//create a thread object for searching
				
				
				destinations = searching.trackingTargets();
				nav.turnToDest(destinations.get(0)[0], destinations.get(0)[1]);  // turn to the first target detected
				this.approachTo(); // approach to the object to be ready for color check
				
				if (lightSensor.colorCheck()) {
					// if target is a styrofoam, then grasp it
					Sound.beep();
					captureFoam(); // ********************************fix later***************************************
					nav.setDest(BOARD_EDGE, BOARD_EDGE); // initialize the dest************************************fix later 
					destinations.remove(0);	   //remove the first target from the collection after detection									
					state = State.TRAVELING;
				} else {
					//// if target is a wooden block, then mark it on the map and travel to next target
					nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
					Sound.twoBeeps();
					destinations.remove(0);	   //remove the first target from the collection after detection									
					state = State.AVOIDING;
				}
				break;
			
			case TRAVELING:	
				if(destinations.isEmpty()){
					state = State.SEARCHING;
					break;
				}
				nav.setDest(destinations.get(0)[0], destinations.get(0)[1]);
				(new Thread(nav)).start();			//create a thread to run travelTo in nav
				if(checkFront()){					//when robot encounter an object 
					nav.interruptTraveling();				
					Sound.twoBeeps();
					approachTo();
					this.approachTo(); // approach to the object to be ready for color check
					
					if (lightSensor.colorCheck()) {
						// if target is a styrofoam, then grasp it
						Sound.beep();
						captureFoam(); // ********************************fix later***************************************
						nav.setDest(BOARD_EDGE, BOARD_EDGE); // initialize the dest************************************fix later 
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.TRAVELING;
					}else {
						//// if target is a wooden block, then mark it on the map and travel to next target
						nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
						Sound.twoBeeps();
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.AVOIDING;
						}
				}
				break;
			
			case AVOIDING:
				/*if (nav.isTraveling()){
					Avoidance avoidance = new Avoidance(nav, leftUS, rightUS);
					avoidance.start();	
					try {
						avoidance.join();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}		// avoidance joins the main thread and main thread continue after avoidance finish
					state = State.TRAVELING;
				}else{*/
					Avoidance avoidance = new Avoidance(nav, leftUS, rightUS);
					avoidance.start();		// when robot is searching and meets block, simply avoid it
					try {
						avoidance.join();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					state = State.TRAVELING;
				//}
				break;

			default:
				break;
			}
		}
	}
	
	private void captureFoam(){
		nav.revert();	
		nav.turn(180);    //rotate 180 to get back side facing the foam
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
	private boolean checkFront(){
		if(frontUS.readUSDistance() < TARGET_DISTANCE){
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
