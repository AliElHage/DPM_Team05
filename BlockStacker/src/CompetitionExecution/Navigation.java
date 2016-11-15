package CompetitionExecution;


/*
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * moves robot
 * @author courtneywright
 *
 */
public class Navigation {
	final static int FAST = 175, SLOW = 60, ACCELERATION = 4000, TURN_SPEED = 75;    //SLOW =60  FAST =100
	final static double DEG_ERR = 1.0, CM_ERR = 1.0;
	public Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double desiredX, desiredY;
	private boolean interrupted, isTraveling;
	private boolean movingFront, movingRight;

	public Navigation(Odometer odo) {
		this.odometer = odo;
		this.interrupted = false;
		this.isTraveling = false;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public synchronized void travelTo(double x, double y) {
		double minAng;
		
		this.isTraveling = true;
		desiredX = x;
		desiredY = y;
		
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			if(!interrupted){
				minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
				if (minAng < 0)
					minAng += 360.0;
				this.turnTo(minAng, false);
				this.setSpeeds(FAST, FAST);
			}else{
				return;				//exit the method when traveling is interrupted
			}
		}
		this.setSpeeds(0, 0);
		this.isTraveling = false;
	}
	/*
	 * TravelTo function that behaves just like TravelTo except it goes along the 2 sides
	 * of the triangle
	 */
	public synchronized void travelToRightAngle(double x, double y) {
		double minAng;
		
		this.isTraveling = true;
		desiredX = x;
		desiredY = y;
		
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			if(!interrupted){
				minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
				if (minAng < 0)
					minAng += 360.0;
				this.turnTo(minAng, false);
				
				//turn to face horizontal side of triangle
				//angle to turn to
				double ang;
				if(x==0){
					ang = 0;
				}
				else {
					ang = Math.atan(y/x);
					
				}	
				turnTo(ang, false);
				
				//moves forward while less than error
				while (Math.abs(x - odometer.getX()) > CM_ERR) {
					this.setSpeeds(FAST,FAST);
				}
				//turns to face vertical axis
				this.turnAmount(-90);
				//moves forward again
				while (Math.abs(y - odometer.getY()) > CM_ERR) {
					this.setSpeeds(FAST,FAST);	
				}				
			}else{
				return;				//exit the method when traveling is interrupted
			}
		}
		this.setSpeeds(0, 0);
		this.isTraveling = false;
	}
	/**
	 * determine whether robot should move to the right or left, front or behind 
	 * @param xDesired
	 * @param yDesired
	 */
	private void determineDirection(double xDesired, double yDesired){	
		this.movingFront = false;
		this.movingRight = false;	
		if(yDesired > odometer.getX())	this.movingFront = true; 	
		if(xDesired > odometer.getY())	this.movingRight = true;	
	}
	
	/**
	 * check if robot has reached the destination that was passed to travelTo() last time 
	 * @return
	 */
	public boolean checkDone(){
		return (Math.abs(desiredX - odometer.getX()) < CM_ERR && Math.abs(desiredY - odometer.getY()) < CM_ERR);
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public void turnTo(double angle, boolean stop) {

		angle = Odometer.fixDegAngle(angle);
		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-TURN_SPEED, TURN_SPEED);
			} else if (error < 0.0) {
				this.setSpeeds(FAST, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(TURN_SPEED, -TURN_SPEED);
			} else {
				this.setSpeeds(-TURN_SPEED, TURN_SPEED);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}
	
	/**
	 * robot rotate to the left(-) or right(+) by degree passed to the function 
	 * @param angle
	 */
	public synchronized void turn(double angle) {
		this.turnTo(odometer.getAng()-angle, true);
	}
	
	public void turnCircle(){
		double initialAngle = odometer.getAng();
		turn(-30);
		rotateLeft();
		while(Math.abs(odometer.getAng() - initialAngle) > DEG_ERR);
		this.setSpeeds(0, 0);
		try { Thread.sleep(500); } catch(Exception e){}
	}
	
	public boolean isTraveling(){
		return isTraveling;
	}
	
	/**
	 *  To interrupt the current traveling 
	 */
	public void interruptTraveling(){
		this.stop();
		this.interrupted = true;
		this.isTraveling = false;
	}
	
	public void turnAmount(double amount){
		if(amount>=0) {
			leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, amount), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, amount), true);
		}
		if(amount<0){
			amount = Math.abs(amount);
			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, amount), true);
			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, amount), true);
		}	
	}
	
	/**
	 *  To resume last traveling
	 */
	public void resumeTraveling(){
		this.interrupted = false;
		(new Thread() {					//start a thread to drive robot to destination
			public void run() {
				travelTo(desiredX, desiredY);  
			}
		}).start();
	}
	
	/**
	 * Go forward a set distance in cm, and stop at the end
	 */
	public void goForward(double distance) {
		this.leftMotor.setSpeed(FAST);
		this.rightMotor.setSpeed(FAST);
		
		this.leftMotor.rotate(convertDistance(Main.WHEEL_RADIUS, distance), true);
		this.rightMotor.rotate(convertDistance(Main.WHEEL_RADIUS, distance), false);
	}
	
	/**
	 * Robot simply rotate to the left without stopping
	 */
	public void rotateLeft() {
		this.setSpeeds(-SLOW, SLOW);
	}
	
	/**
	 * Robot simply rotate to the right without stopping
	 */
	public void rotateRight() {
		this.setSpeeds(SLOW, -SLOW);
	}
	
	public void moveForward(){
		this.setSpeeds(FAST, FAST);
	}
	
	/**
	 * revert robot for certain amount of distance
	 */
	public void revert(){
		this.setSpeeds(-FAST, -FAST);
		try { Thread.sleep(1500); } catch(Exception e){}
		this.stop();
	}
	
	public void stop(){
		this.setSpeeds(0, 0);
	}
	
	//methods originally from SquareDriver class in Lab2
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

} 
