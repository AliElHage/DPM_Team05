package CompetitionExecution;


import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.utility.Delay;


/**
 * This searching class will run as a thread to keeps rotating the robot for updating robot vision, 
 * and this thread is supposed to stop when robot has detected something in main thread,
 * or the robot will keeps rotating in the while loop
 * 
 * @author joey
 *
 */
public class Searching extends Thread{
	
	final static int ACCELERATION=4000, SPEED_NORMAL=200;
	final static int VISION_RANGE=90, VISION_ANGLE_START=340, OBJECT_DIS=40;
	final static int TARGET_NUM = 3, FILTER_OUT = 40, FRONT_SIDE_ERR = 5;	
	private Navigation nav;
	private USPoller frontUS, rightSensor;
	private int filterControl;
	private boolean searchingDone;
	private ArrayList<double[]> targets; 				//store the results after sweeping search
	
	
	
	public Searching(Navigation nav, USPoller frontUS, USPoller rightSensor){
		this.nav = nav;
		this.frontUS = frontUS;
		this.rightSensor = rightSensor;
		this.filterControl = 0;
		this.searchingDone = false;
		targets = new ArrayList<>();
	}
	
	public void run(){
		double startingAngle = nav.odometer.getAng();
		nav.rotateLeft();  		
		Delay.msDelay(2000);     // let robot first rotates certain amount of degree for later checking if it has turned 360
		
		while(!searchingDone){
			if(Math.abs(nav.odometer.getAng() - startingAngle) < Navigation.DEG_ERR){
				nav.stopMoving();
				//nav.scoutZone();	// move to next spot to continue searching if robot hasn't found enough targets after sweeping 
				startingAngle = nav.odometer.getAng();		// update starting angle
				nav.rotateLeft();							// starting sweeping again 
				Delay.msDelay(2000);
			}
		}
	}
	
	
	/**
	 * The robot will rotate and sweep around with frontUS to track 3 targets
	 * @return ArrayList<double []> contains x and y value of targets 
	 */
	public ArrayList<double []> trackingTargets(){
		double targetDistance, targetAngle;
		//boolean filterOn = true;		//by switching filter on and off to ensure robot not to record the same target
		
		
		
		while(targets.size()<TARGET_NUM){			//store 3 target for each sweeping search 
			targetDistance = frontUS.getFilteredValue(OBJECT_DIS, FILTER_OUT);
			targetAngle = nav.odometer.getAng();
			targets.add(new double[] {targetDistance, targetAngle});
			
			
			
			//filter for obeject detection 
	/*		if (distance < OBJECT_DIS && filterControl < FILTER_OUT) {
				// when robot first gets cut off value for detection of an object, ignore them at first 
				filterControl++;
			} else if(distance < OBJECT_DIS && filterOn) {
				// when robot keep getting the cut off value for detection of an object, record the distance and theata for localization of the target
				System.out.println("       "+(int)distance+" "+(int)nav.odometer.getAng());
				targets.add(new double[] {distance, nav.odometer.getAng()});
				filterOn = false;			//once robot has record a target, turn filter off
				Sound.beep();
			} else if(distance > OBJECT_DIS){
				// when robot reads distance greater than cut off value again, reset filter to detect next target
				filterControl = 0;
				filterOn = true;			
			}*/
		}
		return this.getDestSet(targets);		//convert the distance and angle to x and y of targets to return 
	}
	
	
	public void stopSeaching(){
		this.searchingDone = true;
	}
	
	/**
	 * To get the dest coordinates by distance and angle when robot detect a target
	 * @param distance to the traget 
	 * @param angle from odometer reading when facing the traget
	 * @return x and y value of destination 
	 */
	private double[] getDest(double dis, double angle){
		return new double[] {nav.odometer.getX()+dis*Math.cos(angle), nav.odometer.getY()+dis*Math.sin(angle)};
	}
	
	
	/**
	 * 
	 * @param targets includes distance and angles to each target
	 * @return ArrayList<double[]> contains x and y value of the target 
	 */
	private ArrayList<double[]> getDestSet(ArrayList<double[]> targets){			//*******************bug here************/
		ArrayList<double[]> dests = new ArrayList<>();
		for(double[] target:targets){
			dests.add(getDest(target[0], target[1]));
		}
		return dests;
	}
	
	/**
	 * To detect object in the front is an obstacle or not
	 * @return booleann
	 */
	public boolean isObstacle(){
		Delay.msDelay(100); 	// wait 100ms to set up US Sensor 
		double lowerFront = frontUS.readUSDistance();
		nav.turn(-90);     		// rotate the robot to check the object by right side US sensor 
		Delay.msDelay(100); 	// wait 100ms to set up US Sensor 
		double higherRight = rightSensor.readUSDistance();
		return Math.abs(lowerFront-higherRight) < FRONT_SIDE_ERR; // return object is an obstacle if the difference if within error 
	}
	
}