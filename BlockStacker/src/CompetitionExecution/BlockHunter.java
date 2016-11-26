package CompetitionExecution;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Delay;


public class BlockHunter extends Thread{
	
	final static int ACCELERATION=4000, SPEED_NORMAL=200;
	final static double TARGET_DISTANCE=12.5, OBJECT_DIS=40, GRASP_DIS= 6.7;
	final static double  DETECTION_OFFSET= 4.3;
	private Navigation nav;
	private USPoller frontUS, leftUS, rightUS;
	private ClawHandler claw; 
	private boolean foamCaptured;
	private boolean scanDone;
	private ArrayList<double[]> destinations;				//store the target coordinates after sweeping search
	
	public BlockHunter(Navigation nav, USPoller frontUS, USPoller leftUS, USPoller rightUS, ClawHandler claw){
		this.nav = nav;
		this.frontUS = frontUS;
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.claw = claw;
		this.scanDone = false;
		this.foamCaptured = false;
		this.destinations = new ArrayList<>();

	}
	
	enum State {SEARCHING, TRAVELING, AVOIDING}		//define three states of robot when it is hunting
	
	public void run(){
		State state = State.SEARCHING;
		
		while(true){
			switch (state) {
			
			case SEARCHING:
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
					state = State.TRAVELING;
				} else {
					//// if target is a wooden block, then mark it on the map and travel to next target
					nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
					Sound.beepSequence();
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
					this.approachTo(); // approach to the object to be ready for color check
					
					if (!this.isObstacle()) {
						// if target is a styrofoam, then grasp it
						Sound.beep();
						claw.grasp(); 
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.TRAVELING;
					}else {
						//// if target is a wooden block, then mark it on the map and travel to next target
						nav.map.markBlocked(destinations.get(0)[0], destinations.get(0)[1]);
						Sound.twoBeeps();
						destinations.remove(0);	   //remove the first target from the collection after detection									
						state = State.TRAVELING;
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
	
	
	/**
	 * Drive robot to have a ideal distance to object in the front to be ready for object detection
	 */
	public void approachTo(){
		double distance, angle, currentDis;
		
		(new Thread(){
			public void run(){
				nav.turn(-20, Navigation.SACNNNG_SPEED);
				nav.turn(40, Navigation.SACNNNG_SPEED);
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
		if(frontUS.readUSDistance() < OBJECT_DIS){
			return true;
		}else{
			return false;
		}
	}
	
	/**
	 * To detect object in the front is an obstacle or not
	 * @return boolean
	 */
	public boolean isObstacle(){
		nav.turn(-90);     		// rotate the robot to check the object by right side US sensor
		nav.goBackward(DETECTION_OFFSET);//let robot move backward a bit to ensure robot to detect the same spot as front
		Delay.msDelay(100); 	// wait 100ms to set up US Sensor 
		return rightUS.readUSDistance() <= TARGET_DISTANCE; // return object is an obstacle if the reading less than target distance 
	}
	
	/**
	 * Adjust robot position to grasp the foam block
	 */
	public void goToFoam(){
		nav.turn(90);
		if(frontUS.readUSDistance() > GRASP_DIS){
			nav.moveForward();
		}
		nav.stopMoving();
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
