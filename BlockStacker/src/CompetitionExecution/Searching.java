package blockStacker;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This searching class will run as a thread to keeps rotating the robot for updating robot vision, 
 * and this thread is supposed to stop when robot has detected something in main thread,
 * or the robot will keeps rotating in the while loop
 * 
 * @author joey
 *
 */
public class Searching extends Thread{
	
	final static int UPDATING_ANGLE=3, ACCELERATION=4000, SPEED_NORMAL=200;
	final static int SAFE_DISTANCE = 10, VISION_RANGE=90, VISION_ANGLE_START=340;
	private Navigation nav;
	
	public Searching(Navigation nav){
		this.nav = nav;
	}
	
	public void run(){
		nav.turnTo(0, true);			//ensure each searching will start at position 0
		while(true){
			nav.turn(-VISION_RANGE);
			nav.turnTo(90, true);   			 // robot rotates to 90 along the positive y axis 
			nav.goForward(SAFE_DISTANCE);		//move a bit forward to a new location and start searching again
			nav.turnTo(VISION_ANGLE_START,true);
		}

	}
}