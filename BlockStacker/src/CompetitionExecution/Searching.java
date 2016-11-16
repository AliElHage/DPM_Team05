package CompetitionExecution;


import java.util.ArrayList;

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
	private USPoller frontUS;
	private volatile boolean interrupted;
	ArrayList<Double[]> targets; 				//store the results after sweeping search
	ArrayList<Double[]> dests;				//store the target coordinates after sweeping search
	
	public Searching(Navigation nav, USPoller frontUS){
		this.nav = nav;
		this.frontUS = frontUS;
		this.interrupted = false;   	//
		targets = new ArrayList<>();
	}
	
	public void run(){
		nav.turnTo(0, true);			//ensure each searching will start at position 0
		nav.rotateLeft();				//set robot keeping rotating to left
		while(nav.odometer.getAng()< VISION_RANGE){}	// thread stalls here until robot has rotated by VISION_RANGE
		nav.stopMoving();
		
			/*nav.turnTo(90, true);   			 // robot rotates to 90 along the positive y axis 
			nav.goForward(SAFE_DISTANCE);		//move a bit forward to a new location and start searching again
			nav.turnTo(VISION_ANGLE_START,true);*/
	}
	
	
	
	
	/**
	 * To get the dest coordinates by distance and angle when robot detect a target
	 * @param dis
	 * @param angle
	 * @return x and y value of destination 
	 */
	private double[] getDest(double dis, double angle){
		return new double[] {nav.odometer.getX()+dis*Math.cos(angle), nav.odometer.getY()+dis*Math.sin(angle)};
	}
	
	private ArrayList<Double[]> getDestSet(ArrayList<Double[]> targets){
		ArrayList<Double[]> dests = new ArrayList<>();
		for(Double[] target:targets){
			dests.add(new Double[]{new Double}));
		}
	}
	
/*	public synchronized void stopThread(){
		this.interrupted = true;
	}*/
}