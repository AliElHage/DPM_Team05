package CompetitionExecution;


import java.awt.List;
import java.util.ArrayList;
import java.util.Arrays;

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
 * Moves robot in any direction desired including forward, backward and turning
 */
public class Navigation extends Thread{
	enum Direction {LEFT,RIGHT,UP,DOWN,UPLEFT,UPRIGHT,DOWNLEFT,DOWNRIGHT}
	
	static final int TURN_SPEED = 150;
	final static int FAST = 200, SLOW = 150, SACNNNG_SPEED = 50, ACCELERATION = 4000;    //SLOW =60  FAST =175  SCAN =45
	public Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double desiredX, desiredY;
	private int xGrid, yGrid;
	private int destGridX, destGridY, currentGridX, currentGridY;
	private boolean interrupted, isTraveling;
	public FieldMap map; 
	public ArrayList<Grid> zoneDesignated;
	final static double DEG_ERR = 2.0, CM_ERR = 2.0;

	/**
	 * Constructor for Navigation class to set up Object that can move robot anywhere desired.
	 * @param odo odometer so that navigation knows where the robot is at all times
	 */
	public Navigation(Odometer odo) {
		this.odometer = odo;
		this.interrupted = false;
		this.isTraveling = false;
		this.map = new FieldMap();
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		
		/**
		 * Set acceleration
		 */
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		/**
		 * Set up the map conditions depending on if the robot is designated to be the 
		 * builder or collector robot. Builder not allowed in red zone and stacker not
		 * allowed in red zone.
		 */
		if(Main.isBuilder){			
			this.zoneDesignated = map.getZone(Main.LGZx, Main.LGZy, Main.UGZx, Main.UGZy);
			map.zoneBlocked(Main.LRZx, Main.LRZy, Main.URZx, Main.URZy);	
		}else{						
			this.zoneDesignated = map.getZone(Main.LRZx, Main.LRZy, Main.URZx, Main.URZy);
			map.zoneBlocked(Main.LGZx, Main.LGZy, Main.UGZx, Main.UGZy);	
		}
	}
	
	public void run(){
		while(!checkDone() && !interrupted){	//keep running following codes if travelTo is not done or not interrupted
			this.travelByPath(desiredX, desiredY);
			//this.travelTo(desiredX, desiredY);
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
	
	/**
	 * To initialized the desiredX and desiredY by passing a dest grid
	 * @param grid
	 */
	public synchronized void setDest(Grid grid){
		double[] dest = FieldMap.convertGridToPoint(grid.getGridX(), grid.getGridY());
		this.desiredX = dest[0];
		this.desiredY = dest[1];
	}
	
	
	private ArrayList<Grid> calculatePath(Grid gridDest){
		boolean isBlocked = false;
		ArrayList<Grid>	path = new ArrayList<Grid>();
		ArrayList<Grid> leftPath = new ArrayList<Grid>(); 	//create two array for taking upper and lower detour respectively
		ArrayList<Grid> rightPath = new ArrayList<Grid>();
		
		for(Grid grid: getPath(gridDest)){
			leftPath.add(grid);
			rightPath.add(grid);
			path.add(grid);
		}
		int leftPathLength = path.size();
		int rightPathLength = path.size();
		
		//skip the first grid
		int i = 0;
		while(i<leftPathLength) {
			Grid grid = leftPath.get(i);
			if(map.checkBlocked(grid) && i>0){
				Grid prevGrid = leftPath.get(i-1);
				isBlocked=true;
				leftPath.remove(i);
				leftPath.addAll(i, getLeftDetour(grid, gridDest, prevGrid));
			}
			else {
				i++;
			}
			leftPathLength = leftPath.size();
		} 
		int j =0;
		while(j<rightPathLength) {	
			Grid grid = rightPath.get(j);			
			if(map.checkBlocked(grid) && j>0){
				Grid prevGrid = rightPath.get(j-1);
				isBlocked=true;
				rightPath.remove(j);
				rightPath.addAll(j, getRightDetour(grid, gridDest, prevGrid));
			}
			else {
				j++;
			}
			rightPathLength = rightPath.size();
		}
		if(!isBlocked) 
			return path;
		
		map.revisePath(leftPath);
		map.revisePath(rightPath);
		if(leftPath.size()<rightPath.size()){	// if taking upper detour covers less grids then return it
			return leftPath;
		}else{									//otherwise return the lower detour
			return rightPath;
		}
	}
	
	/**
	 * This method return a collection of grids decting the upper detour around the grid 
	 * @param gridBlocked
	 * @return an arrayList of grid depicting a detour around the grid
	 */
	public ArrayList<Grid> getLeftDetour(Grid gridBlocked, Grid destGrid, Grid prevGrid){
		ArrayList<Grid> leftPath = new ArrayList<>();
		int gridBlockedX = gridBlocked.getGridX();
		int gridBlockedY = gridBlocked.getGridY();
		int prevGridY = prevGrid.getGridY();
		int prevGridX = prevGrid.getGridX();
		
		if(gridBlockedX == 0 || gridBlockedX == 10 || gridBlockedY == 0 || gridBlockedY == 10) {
			return leftPath;
		}
		if(prevGridY<gridBlockedY) {
			if(prevGridX > gridBlockedX)
				leftPath.add(map.getGrid(gridBlockedX, gridBlockedY -1));
			for(int i=0; i<3;i++) {
				leftPath.add(map.getGrid(gridBlockedX -1, gridBlockedY +(i-1)));
				if(map.getGrid(gridBlockedX -1, gridBlockedY +(i-1)).equals(destGrid))
					return leftPath;
			}
			leftPath.add(map.getGrid(gridBlockedX, gridBlockedY+1));
		}
		else if(prevGridY > gridBlockedY) {
			if(prevGridX < gridBlockedX)
				leftPath.add(map.getGrid(gridBlockedX, gridBlockedY+1));
			for(int i=0;i<3;i++){		
				leftPath.add(map.getGrid(gridBlockedX+1, gridBlockedY - (i-1)));				
				if(map.getGrid(gridBlockedX+1, gridBlockedY-(i-1)).equals(destGrid))
					return leftPath;
			}
			leftPath.add(map.getGrid(gridBlockedX, gridBlockedY-1));
		}
		else if(prevGridX < gridBlockedX) {
			leftPath.add(map.getGrid(gridBlockedX -1, gridBlockedY));
			for(int i=0; i<3;i++) {
				leftPath.add(map.getGrid(gridBlockedX +(i-1), gridBlockedY +1));
				if(map.getGrid(gridBlockedX +(i-1), gridBlockedY +1).equals(destGrid))
					return leftPath;
			}
			leftPath.add(map.getGrid(gridBlockedX +1, gridBlockedY));
		}	
		else {
			leftPath.add(map.getGrid(gridBlockedX +1, gridBlockedY));
			for(int i=0; i<3;i++) {
				leftPath.add(map.getGrid(gridBlockedX -(i-1), gridBlockedY -1));
				if(map.getGrid(gridBlockedX -(i-1), gridBlockedY -1).equals(destGrid))
					return leftPath;
			}
			leftPath.add(map.getGrid(gridBlockedX -1, gridBlockedY));
		}
		return leftPath;
	}
	
	
	/**
	 * This method return a collection of grids decting the lower detour around the grid 
	 * @param gridBlocked
	 * @return an arrayList of grid depicting a detour around the grid
	 */
	public ArrayList<Grid> getRightDetour(Grid gridBlocked, Grid destGrid, Grid prevGrid){
		ArrayList<Grid> rightPath = new ArrayList<>();
		int gridBlockedX = gridBlocked.getGridX();
		int gridBlockedY = gridBlocked.getGridY();
		int prevGridY = prevGrid.getGridY();
		int prevGridX = prevGrid.getGridX();
		if(gridBlockedX == 0 || gridBlockedX == 10 || gridBlockedY == 0 || gridBlockedY == 10) {
			return rightPath;
		}
		
		if(prevGridY<gridBlockedY){
			if(prevGridX < gridBlockedX)
				rightPath.add(map.getGrid(gridBlockedX, gridBlockedY-1));
			for(int i=0; i<3;i++) {
				rightPath.add(map.getGrid(gridBlockedX+1, gridBlockedY +(i-1)));
				if(map.getGrid(gridBlockedX+1, gridBlockedY + (i-1)).equals(destGrid))
					return rightPath;
			}
			rightPath.add(map.getGrid(gridBlockedX, gridBlockedY+1));
		}
		else if(prevGridY>gridBlockedY) {
			if(prevGridX > gridBlockedX )
				rightPath.add(map.getGrid(gridBlockedX, gridBlockedY +1));
			for(int i=0; i<3;i++) {
				rightPath.add(map.getGrid(gridBlockedX -1, gridBlockedY -(i-1)));
				if(map.getGrid(gridBlockedX -1, gridBlockedY -(i-1)).equals(destGrid))
					return rightPath;
			}
			rightPath.add(map.getGrid(gridBlockedX, gridBlockedY-1));
		}
		else if(prevGridX<gridBlockedX){
			rightPath.add(map.getGrid(gridBlockedX-1, gridBlockedY));
			for(int i=0;i<3;i++){		
				rightPath.add(map.getGrid(gridBlockedX + (i-1), gridBlockedY-1));			
				if(map.getGrid(gridBlockedX + (i-1), gridBlockedY-1).equals(destGrid))
					return rightPath;
			}
			rightPath.add(map.getGrid(gridBlockedX+1, gridBlockedY));	
		}
		else {
			rightPath.add(map.getGrid(gridBlockedX+1, gridBlockedY));
			for(int i=0;i<3;i++){		
				rightPath.add(map.getGrid(gridBlockedX-(i-1), gridBlockedY+1));				
				if(map.getGrid(gridBlockedX-(i-1), gridBlockedY+1).equals(destGrid))
					return rightPath;
			}
			rightPath.add(map.getGrid(gridBlockedX-1, gridBlockedY));	
		}

		return rightPath;
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
		
	
		switch (determineDirection()) {
		case UP:
			path.add(map.getGrid(currentGridX, currentGridY));
			while(currentGridY!=destGridY ){
				LCD.drawString("Up", 0, 2);
				path.add(map.getGrid(currentGridX, ++currentGridY));		// add the grids to the path collection to constitute a path up
			}
			break;
			
		case DOWN:
			path.add(map.getGrid(currentGridX, currentGridY));
			while(currentGridY!=destGridY){
				LCD.drawString("Down", 0, 2);
				path.add(map.getGrid(currentGridX, --currentGridY));		// add the grid to the path collection to constitute a path down
			}
			break;
			
		case LEFT:
			path.add(map.getGrid(currentGridX, currentGridY));
			while(currentGridX!=destGridX){
				LCD.drawString("Left", 0, 2);
				path.add(map.getGrid(--currentGridX, currentGridY));		// add the grid to the path collection to constitute a path left
			}
			break;
			
		case RIGHT:	
			path.add(map.getGrid(currentGridX, currentGridY));
			while(currentGridX!=destGridX){
				LCD.drawString("Right", 0, 2);
				path.add(map.getGrid(++currentGridX, currentGridY));		// add the grid to the path collection to to constitute a path right
			}
			break;
			
		case UPRIGHT:
			path.add(map.getGrid(currentGridX, currentGridY));
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
			path.add(map.getGrid(currentGridX, currentGridY));
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
			path.add(map.getGrid(currentGridX, currentGridY));
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
			path.add(map.getGrid(currentGridX, currentGridY));
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
		destGridX = destGrid.getGridX();
		destGridY = destGrid.getGridY();		//set up the dests
		
		ArrayList<Grid> travelPath = calculatePath(destGrid);
		for(int i=1; i<travelPath.size(); i++){
			if(interrupted){			// if traveling is interrupted, then return
				return;
			}
			Grid grid = travelPath.get(i);
			double[] dest = FieldMap.convertGridToPoint(grid.getGridX(),grid.getGridY());
			travelTo(dest[0],dest[1]);
			//Delay.msDelay(300);
		}
	}
	
	
	/*
	 * TravelTo function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading
	 */	
	public synchronized void travelTo(double x, double y) {
		double distance;
		
		this.isTraveling = true;
		
		this.turnToDest(x, y);
		//Delay.msDelay(500);
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
	private double minAng;
	public synchronized void turnToDest(double x, double y){
		minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0)
			minAng += 360.0;
		this.turnTo(minAng, true);
	}
	
	/**
	 * 
	 * @return the angle the robot turns at every half
	 */
	public double getDesiredAngle(){
		return this.minAng;
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
	 * @return boolean
	 */
	public boolean checkDone(){
		return (Math.abs(desiredX - odometer.getX()) < CM_ERR && Math.abs(desiredY - odometer.getY()) < CM_ERR);
	}
	
	
	/**
	 * check if robot has reached the specific point
	 * @param x
	 * @param y
	 * @return
	 */
	public boolean checkIfDone(double x, double y) {
		return Math.abs(x - odometer.getX()) < CM_ERR
				&& Math.abs(y - odometer.getY()) < CM_ERR;
	}

	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public synchronized void turnTo(double angle, boolean stop) {
		this.turnTo(angle, stop, TURN_SPEED);
	}
	
	
	/**
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public synchronized void turnTo(double angle, boolean stop, int turningSpeed) {

		angle = Odometer.fixDegAngle(angle);
		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-turningSpeed, turningSpeed);
			} else if (error < 0.0) {
				this.setSpeeds(turningSpeed, -turningSpeed);
			} else if (error > 180.0) {
				this.setSpeeds(turningSpeed, -turningSpeed);
			} else {
				this.setSpeeds(-turningSpeed, turningSpeed);
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
	 * robot rotate to the left(-) or right(+) by degree passed to the function 
	 * @param angle
	 * @param turnSpeed
	 */
	public synchronized void turn(double angle, int turnSpeed) {
		this.turnTo(odometer.getAng()-angle, true,turnSpeed);
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
		this.interrupted = true;
		this.isTraveling = false;
		this.stopMoving();
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
	 *  To resume last traveling without stalling the current thread 
	 */
	public synchronized void resumeTraveling(){
		this.interrupted = false;
		(new Thread(){
			public void run(){
				travelTo(desiredX,desiredY);
			}
		}).start();	//start a thread to drive robot to destination
	}
	
	/**
	 *  To resume last traveling by path without stalling the current thread 
	 */
	public synchronized void resumeTravelingByPath(){
		this.interrupted = false;
		(new Thread(this)).start();	//start a thread to drive robot to destination
	}
	
	
	/**
	 * Drive robot to the first grid of the designated zone
	 */
	public void goZoneDesignated(){
		this.setDest(zoneDesignated.get(0));
		new Thread(this).start();
	}
	
	/**
	 * adjust robot heading to the scecond grid of zone
	 */
	public void turnToZoneDesignated(){
		double[] dest = FieldMap.convertGridToPoint(zoneDesignated.get(1).getGridX(), zoneDesignated.get(1).getGridY());
		this.turnToDest(dest[0],dest[1]);
	}
	
	/**
	 * Drive robot to the corner grid near the starting corner 
	 */
	public void goHome(){
		this.setDest(map.getGrid((double)Main.startCorner.getX(), (double)Main.startCorner.getY()));
		new Thread(this).start();
	}

	public void goIntoHome(){
		switch(Main.startCorner.getId()){
		case 1:
			travelTo(-15, -15);
			break;
		
		case 2:
			travelTo(315, -15);
			break;
			
		case 3:
			travelTo(315, 315);
			break;
		
		case 4:
			travelTo(-15, 315);
			break;
		}
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
		this.setSpeeds(SACNNNG_SPEED, -SACNNNG_SPEED);
	}
	
	/**
	 * drive robot forward without stopping 
	 */
	public void moveForward(){
		this.setSpeeds(FAST, FAST);
	}
	
	/**
	 * revert robot for certain amount of distance
	 */
	public void revert(){
		this.setSpeeds(-FAST, -FAST);
		try { Thread.sleep(300); } catch(Exception e){}
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
	public double[] getCurrentGrid(){
		double[] result = new double[2];
		result[0] = currentGridX;
		result[1] = currentGridY;
		return result;
	}
	public double[] getDestGrid(){
		double[] result = new double[2];
		result[0] = destGridX;
		result[1] = destGridY;
		return result;
	}
	
	

} 