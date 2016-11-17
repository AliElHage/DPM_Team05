package CompetitionExecution;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class will run as a thread when robot has detected an obstacle in the front, 
 * and the thread should be given top priority to ensure robot must run through this thread first.
 * The thread is transient, and will die out itself when avoidance is done
 * @author joey
 *
 */
public class Avoidance extends Thread{
	
	private static final int ERR_DIS = 3;
	private static final int  ROTATE_SPEED = 120, ACCELERATION = 2000, MOTOR_STRAIGHT=150, MOTOR_HIGH = 300, MOTOR_LOW = 0;
	private static final int SENSOR_WALL = 12;	 //the measured value of US distance from sensor to wall when robot rotate to position parallel to the wall 	 
	private USPoller leftUS, rightUS;
	private Navigation nav;
	private boolean handle;

	public Avoidance(Navigation nav, USPoller leftUS, USPoller rightUS) {
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.nav = nav;
		this.handle = false;
	}
	

	public void run() {
		
		nav.rotateLeft(); 	//keep rotating to left till robot position parallel to the wall and get US reading 'SENSOR_WALL'
		while(rightUS.readUSDistance() > SENSOR_WALL){} 
		nav.stopMoving();			
		
		
		nav.moveForward();   //set robot to move forward until it cannot see the wall 
		while(rightUS.readUSDistance() > SENSOR_WALL){}
		nav.stopMoving();  	//robot keeps moving forward till it cannot see wall on the right (change in US reading greater than 3)		
		
		handle = true;
	}

	public boolean handled(){
		return handle;
	}
	


}
