package CompetitionExecution;


import java.util.ArrayList;
import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
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
import lejos.utility.Delay;

/**
 * moves robot
 * @author courtneywright
 *
 */
public class Navigation extends Thread{
	enum Direction {LEFT,RIGHT,UP,DOWN,UPLEFT,UPRIGHT,DOWNLEFT,DOWNRIGHT}
	
	static final int TURN_SPEED = 150;
	final static int FAST = 200, SLOW = 150, SACNNNG_SPEED = 80, ACCELERATION = 4000;    //SLOW =60  FAST =175
	public Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double desiredX, desiredY;
	private int destGridX, destGridY, currentGridX, currentGridY;
	private boolean interrupted, isTraveling;
	public FieldMap map; 
	final static double DEG_ERR = 1.0, CM_ERR = 1.0;

	public Navigation(Odometer odo) {
		this.odometer = odo;
		this.interrupted = false;
		this.isTraveling = false;
		this.map = new FieldMap();
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}
	
	public void run(){
		while(!checkDone() && !interrupted){	//keep running following codes if travelTo is not done or not interrupted
			this.travelByPath(desiredX, desiredY);
			this.travelTo(desiredX, desiredY);
		}
	}
	

	/**
	 * To initialized the desiredX and desiredY so robot can run the thread and drive to the destination
	 * @param desiredX
	 * @param desiredY
	 */
	public synchronized void setDest(double desiredX, double desiredY){
		this.desiredX = desiredX;
		this.desiredY = desiredY;
	}
	
	
	private ArrayList<Grid> calculatePath(Grid gridDest){
		int index = 0;
		ArrayList<Grid> prePath1 = this.getPath(gridDest);	//create two array for taking upper and lower detour respectively
		ArrayList<Grid> prePath2 = this.getPath(gridDest);
		for(Grid grid: prePath1){
			if(map.checkBlocked(grid)){
				prePath1.remove(grid);
				prePath2.remove(grid);
				prePath1.addAll(index, map.getUpperDetour(grid));
				prePath2.addAll(index, map.getLowerDetour(grid));
			}
			index++;
		}
		map.revisePath(prePath1);
		map.revisePath(prePath2);
		if(prePath1.size()<prePath2.size()){	// if taking upper detour covers less grids then return it
			return prePath1;
		}else{									//otherwise return the lower detour
			return prePath2;
		}
	}
		
	
	
	/**
	 * This class helps to find a path comprised of a collection of grids the robot should cover to get to dest. 
	 * @param desiredX
	 * @param desiredY
	 * @return ArrayList<Grid>
	 */
	private ArrayList<Grid> getPath(double desiredX, double desiredY){
		ArrayList<Grid> path = getPath(map.getGrid(desiredX, desiredY));
		return path;			// return the constituted path	
	}
	
	
	
	/**
	 * This class helps to find a path comprised of a collection of grids the robot should cover to get to dest.
	 * @param gridDest
	 * @return
	 */
	private ArrayList<Grid> getPath(Grid gridDest){
		int [] currentGridIndex;
		
		ArrayList<Grid> path = new ArrayList<>();
		destGridX = gridDest.getGridX();
		destGridY = gridDest.getGridY();
		currentGridIndex = FieldMap.convertPointToGrid(odometer.getX(), odometer.getY());
		currentGridX = currentGridIndex[0];
		currentGridY = currentGridIndex[1];
		
		//test
		LCD.drawString("curGrid: " +currentGridX +","+ currentGridY,0, 3);
		LCD.drawString("destGrid: " +destGridX +","+ destGridY,0, 4);
		//end test

		
		switch (determineDirection()) {
		case UP:
			while(currentGridY!=destGridY){
				LCD.drawString("Up", 0, 2);
				path.add(map.getGrid(currentGridX, ++currentGridY));		// add the grids to the path collection to constitute a path up
			}
			break;
			
		case DOWN:
			while(currentGridY!=destGridY){
				LCD.drawString("Down", 0, 2);
				path.add(map.getGrid(currentGridX, --currentGridY));		// add the grid to the path collection to constitute a path down
			}
			break;
			
		case LEFT:
			while(currentGridX!=destGridX){
				LCD.drawString("Left", 0, 2);
				path.add(map.getGrid(--currentGridX, currentGridY));		// add the grid to the path collection to constitute a path left
			}
			break;
			
		case RIGHT:	
			while(currentGridX!=destGridX){
				LCD.drawString("Right", 0, 2);
				path.add(map.getGrid(++currentGridX, currentGridY));		// add the grid to the path collection to to constitute a path right
			}
			break;
			
		case UPRIGHT:
			while(currentGridY!=destGridY || currentGridX!=destGridX){
				LCD.drawString("Upright", 0, 2);
				if(destGridY > currentGridY && destGridX > currentGridX){
					path.add(map.getGrid(++currentGridX, ++currentGridY));	// add the grids to the path collection to constitute a path upright
				}else if(destGridY > currentGridY){
					path.add(map.getGrid(currentGridX, ++currentGridY));		// add the grids to the path collection to constitute a path up
				}else{
					path.add(map.getGrid(++currentGridX, currentGridY));		// add the grids to the path collection to constitute a path right
				}
			}
			break;
		
		case UPLEFT:
			while(currentGridY!=destGridY || currentGridX!=destGridX){
				LCD.drawString("Upleft", 0, 2);
				if(destGridY > currentGridY && destGridX < currentGridX){
					path.add(map.getGrid(--currentGridX, ++currentGridY));	// add the grids to the path collection to constitute a path upleft
				}else if(destGridY > currentGridY){
					path.add(map.getGrid(currentGridX, ++currentGridY));		// add the grids to the path collection to constitute a path up
				}else{
					path.add(map.getGrid(--currentGridX, currentGridY));		// add the grids to the path collection to constitute a path left
				}
			}
			break;
			
		case DOWNLEFT:
			while(currentGridY!=destGridY || currentGridX!=destGridX){
				LCD.drawString("DownLeft", 0, 2);
				if(destGridY < currentGridY && destGridX < currentGridX){
					path.add(map.getGrid(--currentGridX, --currentGridY));		// add the grids to the path collection to constitute a path downleft
				}else if(destGridY < currentGridY){
					path.add(map.getGrid(currentGridX, --currentGridY));		// add the grids to the path collection to constitute a path down
				}else{
					path.add(map.getGrid(--currentGridX, currentGridY));		// add the grids to the path collection to constitute a path left
				}
			}
			break;
		
		case DOWNRIGHT:
			while(currentGridY!=destGridY || currentGridX!=destGridX){
				LCD.drawString("DownRight", 0, 2);
				if(destGridY < currentGridY && destGridX > currentGridX){
					path.add(map.getGrid(++currentGridX, --currentGridY));		// add the grids to the path collection to constitute a path downright
				}else if(destGridY < currentGridY){
					path.add(map.getGrid(currentGridX, --currentGridY));		// add the grids to the path collection to constitute a path down
				}else{
					path.add(map.getGrid(++currentGridX, currentGridY));		// add the grids to the path collection to constitute a path right
				}
			}
			break;
			
		default:
			break;
		}
		
		return path;			// return the constituted path	
	}

	/**
	 * determine whether robot should move to the right or left, front or back
	 * @param desiredX
	 * @param desiredY
	 * @return Direction
	 */
	private Direction determineDirection(){	
		//compares grid locations
		//same grid
		if(destGridX==currentGridX && destGridY==currentGridY)
			return Direction.UP;	//arbitrary direction because both grids are the same
		//move right
		if(destGridX>currentGridX) {
			if( destGridY > currentGridY)
				return Direction.UPRIGHT;
			else if( destGridY < currentGridY)
				return Direction.DOWNRIGHT;
			else 
				return Direction.RIGHT;
		}
		//move left
		else if(destGridX<currentGridX){
			if( destGridY> currentGridY)
				return Direction.UPLEFT;
			else if (destGridY < currentGridY )
				return Direction.DOWNLEFT;
			else 
				return Direction.LEFT;
		}
		//move up/down
		else {
			if(destGridY > currentGridY)
				return Direction.UP;
			else
				return Direction.DOWN;
		}
	}
	
	
	/**
	 * determine whether robot should move to the right or left, front or behind
	 * @param desiredX
	 * @param desiredY
	 * @return Direction
	 */
	private Direction determineDirection(double desiredX, double desiredY){	
		Grid destGrid = map.getGrid(desiredX, desiredY);
		Grid currentGrid = map.getGrid(odometer.getX(), odometer.getY());
		destGridX = destGrid.getGridX();
		destGridY = destGrid.getGridY();
		currentGridX = currentGrid.getGridX();
		currentGridY = currentGrid.getGridY();
		return determineDirection();
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
	/**
	 * Drive robot to the destination via the calculated path of grids
	 * @param gridX x index of Grid
	 * @param gridY y index of Grid
	 */
	public void goToGrid(int gridX, int gridY){
		double[] dest = FieldMap.convertGridToPoint(gridX,gridY);
		this.travelTo(dest[0],dest[1]);
	}
	
	public synchronized void scoutZone(ArrayList<Grid> border){
		int index = 0;			//index to keep track of current position on the border 
		int[] currentGrid = FieldMap.convertPointToGrid(odometer.getX(), odometer.getY());
		for(Grid grid: border){
			if(grid.equals(currentGrid)){
				this.travelByPath(border.get( (index+2) % border.size())); 	//travel to next spot to continue scouting
				return;
			}
			index++;
		}
	}
	
	/**
	 * Drive robot to the destination via the calculated path of grids
	 * @param x coordinate of destination point
	 * @param y coordinate of destination point
	 */
	public synchronized void travelByPath(double x, double y){
		Grid destGrid = map.getGrid(x, y);
		this.travelByPath(destGrid);
	}
	
	
	/**
	 * Drive robot to the destination via the calculated path of grids
	 * @param destGrid
	 */
	public void travelByPath(Grid destGrid){
		ArrayList<Grid> travelPath = getPath(destGrid);
		for(Grid grid: travelPath){
			double[] dest = FieldMap.convertGridToPoint(grid.getGridX(),grid.getGridY());
				travelTo(dest[0],dest[1]);
				try {
					Thread.sleep(300);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
		}
	}
	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */	
	public synchronized void travelTo(double x, double y) {
		double distance;
		
		this.isTraveling = true;
		desiredX = x;
		desiredY = y;
		
		this.turnToDest(x, y);
		Delay.msDelay(200);
		// calculate distance to travel with given coordinates
		distance = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()), 2) + Math.pow(Math.abs(y - odometer.getY()), 2));   
		
		this.goForward(distance);  // travel straight the needed distance
		this.setSpeeds(0, 0);
		this.isTraveling = false;
	}
	/*	*********************************
	 * 	The following  is previous TravelTo() written by TA, this method is theoretically more accurate than our TravelTo() above, 
	 * 	but it's problematic as well. Since it contains a while loop keeping checking the error, sometimes the robot can run in 
	 * crash keeping driving without stop. 
	 * 
	 * 	*********************************
	 * 	public synchronized void travelTo(double x, double y) {
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
	 */
	
	
	
	
	/**
	 * Turn robot to face to the destination 
	 * @param x 
	 * @param y
	 */
	public synchronized void turnToDest(double x, double y){
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0)
			minAng += 360.0;
		this.turnTo(minAng, true);
	}
	
	
	
	/**
	 * TravelTo function that behaves just like TravelTo except it goes along the 2 sides
	 * of the triangle
	 * Relies on the robot knowing its angle theta already (ie. localized)
	 */
	public synchronized void travelToRightAngle(double x, double y) {
		//note: we may not need minAng
		//double minAng;
		
		this.isTraveling = true;
		desiredX = x;
		desiredY = y;
		
		//I don't think we need this
/*		minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0)
			minAng += 360.0;
		this.turnTo(minAng, false);		*/
		
		
		double Xangle, Yangle;
		//HORIZONTAL SIDE
		if(x<0)
			Xangle = 180;
		else
			Xangle = 0;
		//Turns to face then travels along x axis
		while (Math.abs(x - odometer.getX()) > CM_ERR) {
			if(!interrupted) {
				this.turnTo(Xangle, false);
				this.setSpeeds(FAST,FAST);
			}
			else return; 	//exit the method when traveling is interrupted
		}
		
		//VERTICAL SIDE
		if(y < 0) 
			Yangle = 90;
		else
			Yangle = 270;
		//Turns to face then travels along y axis
		while (Math.abs(y - odometer.getY()) > CM_ERR) {
			if(!interrupted) {
				this.turnTo(Yangle, false);
				this.setSpeeds(FAST,FAST);
			}
			else return; 	//exit the method when traveling is interrupted
		}							
		this.setSpeeds(0, 0);
		this.isTraveling = false;
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
	
	/**
	 * robot turn 360 degree and stops at the end
	 */
	public void turnCircle(){
		double initialAngle = odometer.getAng();		
		this.rotateLeft();
		Delay.msDelay(2000);   // set up for turning a circle
		while(Math.abs(odometer.getAng() - initialAngle) > DEG_ERR);
		this.setSpeeds(0, 0);
	}
	
	public boolean isTraveling(){
		return isTraveling;
	}
	
	/**
	 *  To interrupt the current traveling 
	 */
	public void interruptTraveling(){
		this.stopMoving();
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
		(new Thread(this)).start();	//start a thread to drive robot to destination
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
	 * Go backward a set distance in cm, and stop at the end
	 */
	public void goBackward(double distance) {
		this.leftMotor.setSpeed(FAST);
		this.rightMotor.setSpeed(FAST);
		this.leftMotor.rotate(-convertDistance(Main.WHEEL_RADIUS, distance), true);
		this.rightMotor.rotate(-convertDistance(Main.WHEEL_RADIUS, distance), false);
	}
	
	
	/**
	 * Robot simply rotate to the left specifically used for scanning objects
	 */
	public void rotateLeft() {
		this.setSpeeds(-SACNNNG_SPEED, SACNNNG_SPEED);
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
		this.stopMoving();
	}
	
	public void stopMoving(){
		this.setSpeeds(0, 0);
	}
	
	public FieldMap getFieldMap(){
		return this.map;
	}
	
	//methods originally from SquareDriver class in Lab2
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public double getDesiredX() {
		return desiredX;
	}

	public double getDesiredY() {
		return desiredY;
	}
	
	

} 