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
	
	final static int OBJECT_DIS=40;
	final static int TARGET_NUM = 3, FILTER_OUT = 7; 
	private Navigation nav;
	private static USPoller frontUS, rightUS;
	private boolean searchingDone;
	private ArrayList<double[]> targets; 				//store the results after sweeping search
	
	public Searching(Navigation nav, USPoller frontUS, USPoller rightSensor){
		this.nav = nav;
		this.frontUS = frontUS;
		this.rightUS = rightSensor;
		this.searchingDone = false;
		targets = new ArrayList<>();
	}
	
	public void run(){
		while(!searchingDone){
			nav.turnCircle(); // let robot first rotates certain amount of degree for later checking if it has turned 360
			//nav.scoutZone();	// move to next spot to continue searching if robot hasn't found enough targets after sweeping 
			nav.goForward(15);		//***********************************************************	
		}
	}
	
	
	/**
	 * The robot will rotate and sweep around with frontUS to track 3 targets
	 * @return ArrayList<double []> contains x and y value of targets 
	 */
	public ArrayList<double []> trackingTargets(){
		double targetFallingDis, targetFallingAngle, targetRisingDis, targetRisingAngle;
		
		while(targets.size()<TARGET_NUM){			//store 3 target for each sweeping search 
			targetFallingDis = frontUS.getFallingEdge(OBJECT_DIS, FILTER_OUT); //record the falling edge distance
			Sound.beep();
			targetFallingAngle = nav.odometer.getAng();		//record the falling edge angle
			targetRisingDis =  frontUS.getRisingEdge(OBJECT_DIS, 2); //record the Rising edge distance
			targetRisingAngle = nav.odometer.getAng();		//record the falling edge angle
			// take the average of falling and rising edge distance and angle to store
			targets.add(new double[] {(targetFallingDis+targetRisingDis)/2, (targetFallingAngle+targetRisingAngle)/2});
			Sound.beepSequence();
			
/*			Delay.msDelay(500);      //ensure robot to record the position of the center of target after a value returned 
			targetFallingAngle = nav.odometer.getAng();		//record the angle;
			targetFallingDis = frontUS.readUSDistance();   //update the distance
			targets.add(new double[] {targetFallingDis, targetFallingAngle});
			Sound.beep();
			Delay.msDelay(1500);	//ensure robot not to record the same target*/
		}
		return this.getDestSet(targets);		//convert the distance and angle to x and y of targets to return 
	}
	
	
	/**
	 * stop the searching thread 
	 */
	public void stopSearching(){
		this.searchingDone = true;
		Delay.msDelay(100);
		nav.stopMoving();
	}
	
	
	/**
	 * To get the dest coordinates by distance and angle when robot detect a target
	 * @param distance to the target 
	 * @param angle from odometer reading when facing the target
	 * @return x and y value of destination 
	 */
	private double[] getDest(double dis, double angle){
		return new double[] {nav.odometer.getX()+dis*Math.cos(Math.toRadians(angle)), 
				nav.odometer.getY()+dis*Math.sin(Math.toRadians(angle))};
	}
	
	
	/**
	 * 
	 * @param targets includes distance and angles to each target
	 * @return ArrayList<double[]> contains x and y value of the target 
	 */
	private ArrayList<double[]> getDestSet(ArrayList<double[]> targets){			
		ArrayList<double[]> dests = new ArrayList<>();
		for(double[] target:targets){
			dests.add(getDest(target[0], target[1]));
		}
		return dests;
	}
	

	
}