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
	private static final int CERTAIN_DIS = 5;  // the measured distance for robot to go forward to see the wall again 
	private USPoller leftUS, rightUS;
	private Navigation nav;
	private boolean handle;

	public Avoidance(Navigation nav, USPoller leftUS, USPoller rightUS) {
		//Default Constructor
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.nav = nav;
		this.handle = false;
	}
	

	public void run() {
		double startTheta, thetaTurned;
		
		startTheta = nav.odometer.getAng();
		while(rightUS.readUSDistance() > SENSOR_WALL){  
			nav.rotateLeft();			//keep rotating to left till robot position parallel to the wall and get US reading 'SENSOR_WALL'
		}
		nav.stop();
		thetaTurned = nav.odometer.getAng() - startTheta; // record how many degree robot has rotate to get parallel position to the wall
		
		nav.turn(thetaTurned);   		//restore robot original position for moving forward after steering clear of obstacle
		nav.goForward(CERTAIN_DIS);      //move forward to see the wall again 
		while(Math.abs(rightUS.readUSDistance()-SENSOR_WALL)< ERR_DIS){
			nav.moveForward();  	//robot keeps moving forward till it cannot see wall on the right (change in US reading greater than 3)			
		}
		nav.stop();
		
		
		handle = true;
	}

	public boolean handled(){
		return handle;
	}
	


}
