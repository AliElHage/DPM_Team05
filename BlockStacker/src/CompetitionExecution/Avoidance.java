package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
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
	private USPoller leftUS, rightUS, frontUS;
	private Navigation nav;
	private boolean handle;
	
	private Odometer odo;
	private final int bandCenter = 30;			//distance robot is to stay away from the block
	private final int bandwidth = 2;			//amount off the bandCetner the robot can be
	private final int FILTER_OUT = 30;
	private static double OGtheta = 0.0, thetaEnd;		//stores angle robot is traveling at before it avoids the block
	private double rightDist, frontDist, error;
	private boolean avoid;
	static double Xstart = 0;
	static double Ystart = 0;
	static double Tstart = 0;
	private static Boolean Avoiding = true;
	private boolean firstAdjust = true;			//checks if this is the original encounter of the block

	public Avoidance(Navigation nav, Odometer odo, USPoller leftUS, USPoller rightUS, USPoller frontUS) {
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		this.frontUS = frontUS;
		this.nav = nav;
		this.odo = odo;
		this.handle = false;
	}
	

//	public void run() {
//		
//		nav.rotateLeft(); 	//keep rotating to left till robot position parallel to the wall and get US reading 'SENSOR_WALL'
//		while(rightUS.readUSDistance() > SENSOR_WALL){} 
//		nav.stopMoving();			
//		
//		
//		nav.moveForward();   //set robot to move forward until it cannot see the wall 
//		while(rightUS.readUSDistance() > SENSOR_WALL){}
//		nav.stopMoving();  	//robot keeps moving forward till it cannot see wall on the right (change in US reading greater than 3)		
//		
//		handle = true;
//	}
//
//	public boolean handled(){
//		return handle;
//	}
	
	public void run() {
		rightDist = rightUS.readUSDistance();
		frontDist = frontUS.readUSDistance();
		error = rightDist-bandCenter;
		
		if(frontDist <= bandCenter) {
			nav.turn(90);
		}
		
		/**
		 * checks if the robot is closer to a block than we would like
		 * if it is, boolean avoid becomes true and next loop beings
		 * as the robot tries to avoid the block
		 */
		if (rightDist <= bandCenter) {
			Sound.beepSequenceUp();
			nav.interruptTraveling();
			avoid = true;
			Xstart = odo.getX();
			Ystart = odo.getY();
			Tstart = odo.getAng();
		}
		
		thetaEnd  = (OGtheta + 3*Math.PI/2) % (2*Math.PI);
		
		
		/**
		 * only goes into this loop if the us sensor polls a distance closer
		 * than the bandCenter. This loop is starts the BangBang process to avoid the block
		 * and (tries to) kick out when the robot fully passes the block
		 */
		if (avoid) {
			double thetaEnd  = (OGtheta + 3*Math.PI/2) % (2*Math.PI);
			
//			//TEST
//				int degOG = (int)(OGtheta * 180/Math.PI);
//				int ED = (int)(thetaEnd * 180/Math.PI);
//				LCD.drawString("OG: " + degOG, 10, 3);
//				LCD.drawString("ED: " + ED, 10, 5);
//			//END TEST	
				
				//same as Lab1, moving forward
				if (Math.abs(error) <= bandwidth) {
					nav.goForward(20);
				}
				
				/**
				 * the robot will travel around the block to the right, so if the robot
				 * has made it all the way around the block, the angle read by the
				 * odometer should be 90 degrees less than the angle that the robot was
				 * traveling at before it started navigating around the block. At this 
				 * point, the moveTo method in navigate will (hopefully) be called to
				 * move the robot from its current position to its original destination
				 */
				
				else if (odo.getAng() <= ( thetaEnd + Math.PI/36)  && odo.getAng() >= (  thetaEnd - Math.PI/36)  ) {	
					
					/**
					 * if the original angle at which the robot was traveling is zero
					 * taking into account error, send the robot from where it is after
					 * moving around the block, to the first point, (60,0). If the original
					 * angle at which the robot was traveling is anything other than that,
					 * send it to the second point, (0,60).
					 */
					if (Math.toDegrees(OGtheta) > -5 && Math.toDegrees(OGtheta) < 5){
						LCD.drawString("INSIDE1", 10, 2);
						nav.travelTo(0, 60);
						Avoiding = false;
					} else {
						LCD.drawString("INSIDE2", 10, 2);
						nav.travelTo(60, 0);
						Avoiding = false;
					}
				} 
				
				else if (error < 0) {
					
					/**
					 * keeps the robot a bit closer to the block so it goes around properly
					 */
					if (rightDist < bandCenter-5) {
						
						/**
						 * Checks to see if this is the original encounter of the block using boolean
						 * "firstAdjust". If it is, it sets "OGtheta" to the
						 * angle the robot is currently traveling at and sets
						 * the boolean to false so that "OGtheta" is never reset.
						 */				
						if (firstAdjust) {	
							LCD.drawString("INSIDE", 10, 2);		 
							OGtheta = odo.getAng();			
							firstAdjust = false;					
						}	
						
						/**
						 * same as Lab1, turns robot to the left to avoid the block
						 */
						while (error < 0) {
							nav.turn(1);
						}
					}
			
				} 
				
				/**
				 * same as Lab1 except for "firstAdjust" condition. This statement
				 * turns the robot to the left to go back around to the other side
				 * of the block. The difference is that it checks to make sure that
				 * this is not the first encounter with the block as a double check
				 * to make sure that the robot will never go closer to an object
				 * that it was not trying to avoid (example: drive toward wall).
				 */
				else if (error > 0 && !firstAdjust) {
					
					while (error > 0) {
						nav.turn(1);
					}
//					leftMotor.setSpeed(motorLow+15);
//					rightMotor.setSpeed(motorHigh);
//			
//					leftMotor.forward();
//					rightMotor.forward();
				}
		}
		
		nav.resumeTraveling();
	}

}
