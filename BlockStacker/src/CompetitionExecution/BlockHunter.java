package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * searches for block in competition area
 * @author courtneywright
 *
 */
public class BlockHunter extends Thread{
	
	final static int UPDATING_ANGLE=3, ACCELERATION=4000, SPEED_NORMAL=200, VISION_ANGLE_END=85, VISION_ANGLE_START=340;
	final static double TARGET_DISTANCE=6.4, VISION_RANGE=30, BOARD_EDGE=61, SAFE_DISTANCE=18.0, SAFE_ANGLE=45.0;
	private Odometer odo;
	private Navigation nav;
	private BlockDetection light;
	private USPoller usPoller;
	private EV3LargeRegulatedMotor hookMotorL, hookMotorR;
	private boolean foamCaptured;
	private boolean blockFlag = false;
	
	public BlockHunter(Odometer odometer, Navigation nav, USPoller USPoller, BlockDetection light, EV3LargeRegulatedMotor hookMotorL, EV3LargeRegulatedMotor hookMotorR){
		this.odo = odometer;
		this.nav = nav;
		this.usPoller = USPoller;
		this.light = light;
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
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {}
		for(int i = 0; i < 18; i++){
			nav.turn(-5);
			try {
				Thread.sleep(300);
			} catch (InterruptedException e) {}
			if(checkObject()){
				nav.turn(-10);
				break;
			}
		}
		System.out.println("      FOUND");
		approachTo();
		if(light.colorCheck()){
			Sound.beep();
			captureFoam();
			nav.travelTo(BOARD_EDGE, BOARD_EDGE);
			nav.turnTo(45, true);
			nav.turn(-180);
			return;
		}
		else{
			Sound.twoBeeps();
			avoidBlock();
		}
		
//		
	}
	
	private void captureFoam(){
		nav.revert();	
		nav.turn(180);    //rotate 180 to get back side facing the foam
		//*********motor rotate ...*******************************************************
		hookMotorL.rotate(200, true);
		hookMotorR.rotate(200, false);
		this.foamCaptured = true;
	}

	
	/**
	 *  To avoid the block assuming at distance of TARGET_DISTANCE
	 */
	private void avoidBlock(){
		int count = 0;
		
		//nav.revert();   	//drive robot back a bit to avoid obstacle 
		while(true){//*********************************************************************might change the const here**
			if(usPoller.readUSDistance()< SAFE_DISTANCE){
				nav.turn(-UPDATING_ANGLE);			// robot continuously tweak its heading to the left until its sees nothing in the front
				count += UPDATING_ANGLE;
			}else{
				break;
			}
		}
		nav.turn(-SAFE_ANGLE);
		nav.goForward(SAFE_DISTANCE*2 - 10);
		////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		nav.turn(30);
		for(int i = 0; i < (count - 35) / UPDATING_ANGLE; i++){
			nav.turn(3);
			try {
				Thread.sleep(300);
			} catch (InterruptedException e) {}
			if(checkObject()){
				nav.turn(10);
				blockFlag = true;
				break;
			}
		}
		if(blockFlag){
			System.out.println("      FOUND");
			approachTo();
			if(light.colorCheck()){
				System.out.println("      HEY");
				Sound.beep();
				captureFoam();
				nav.travelTo(BOARD_EDGE, BOARD_EDGE);
				return;
			}
		}
		nav.goForward(SAFE_DISTANCE*2 - 10);
		// again
		for(int i = 0; i < (count - 30) / UPDATING_ANGLE; i++){
			nav.turn(3);
			try {
				Thread.sleep(300);
			} catch (InterruptedException e) {}
			if(checkObject()){
				nav.turn(10);
				blockFlag = true;
				break;
			}
		}
		if(blockFlag){
			System.out.println("      FOUND");
			approachTo();
			if(light.colorCheck()){
				System.out.println("      HEY");
				Sound.beep();
				captureFoam();
				nav.travelTo(BOARD_EDGE, BOARD_EDGE);
				nav.turnTo(45, true);
				nav.turn(-180);
				return;
			}
		}
		////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		//nav.turn(count);			//turn back to original heading 
//		if(usPoller.readUSDistance() > SAFE_DISTANCE){
//			nav.goForward(SAFE_DISTANCE);
//		}
	}
	
	
	/**
	 * Search if there is any object within vision, and approach it to be ready for color check if there is any 
	 */
	private void trackingObject(){
		
		while(true){
			if(usPoller.readUSDistance() > VISION_RANGE){
				System.out.println("         "+(int)usPoller.readUSDistance());
				// the following codes will run when usSensor detects nothing
				nav.turn(-UPDATING_ANGLE); // every loop the robot rotate to left by 5 degree to update vision 
			}else{
				break;
			}
			
			if(odo.getAng()>VISION_ANGLE_END){
				nav.turnTo(90, true);    // robot rotate back to the left side of robot and start a new cycle of vision detection 
				nav.goForward(SAFE_DISTANCE/2);		//move a bit forward to start a new searching
				nav.turnTo(VISION_ANGLE_START,true);
			}
		}
		// when usSensor detects something within vision, approach it to be ready for color check 
		nav.turn(-SAFE_ANGLE);				// to adjust robot heading to the center of object, as it is facing to the edge of object right now.
		this.approachTo();
	}
	
	
	/**
	 *  Check if there is any object within vision range
	 * @return
	 */
	private boolean checkObject(){
		if(usPoller.readUSDistance() < VISION_RANGE){
			System.out.println("       ::" + (int)usPoller.readUSDistance());
			return true;
		}else{
			return false;
		}
	}
	
	/**
	 * Drive robot to have a ideal distance to object in the fron to be ready for color check
	 */
	private void approachTo(){
		while(usPoller.readUSDistance() > TARGET_DISTANCE){
			nav.moveForward();
		}
		nav.stop();
	}
	
	
	
	public boolean obstacleTesting(){
		if(usPoller.readUSDistance() < TARGET_DISTANCE){
			System.out.println("Object Detected");
			return true;
		}
		else{
			return false;
		}
	}
}
