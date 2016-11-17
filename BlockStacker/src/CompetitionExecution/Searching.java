package CompetitionExecution;


import java.util.ArrayList;


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
	final static int VISION_RANGE=90, VISION_ANGLE_START=340, OBJECT_DIS=25;
	final static int TARGET_NUM = 3, FILTER_OUT = 15;	
	private Navigation nav;
	private USPoller frontUS;
	private int filterControl;
	ArrayList<double[]> targets; 				//store the results after sweeping search
	ArrayList<double[]> destions;				//store the target coordinates after sweeping search
	
	public Searching(Navigation nav, USPoller frontUS){
		this.nav = nav;
		this.frontUS = frontUS; 	
		this.filterControl = 0;
		targets = new ArrayList<>();
	}
	
	public void run(){
		nav.turnTo(0, true);			//ensure each searching will start at position 0
		nav.rotateLeft();				//set robot keeping rotating to left
		this.trackingTargets();
		nav.stop();
			/*nav.turnTo(90, true);   			 // robot rotates to 90 along the positive y axis 
			nav.goForward(SAFE_DISTANCE);		//move a bit forward to a new location and start searching again
			nav.turnTo(VISION_ANGLE_START,true);*/
	}
	
	
	/**
	 * The robot will rotate and sweep around with frontUS to track 3 targets
	 * @return ArrayList<double []> contains x and y value of targets 
	 */
	public ArrayList<double []> trackingTargets(){
		double distance;
		ArrayList<double []> targets = new ArrayList<>();
		
		while(targets.size()<TARGET_NUM){			//store 3 target for each sweeping search 
			distance = frontUS.readUSDistance();	 	
			
			//filter for obejct detection 
			if (distance < OBJECT_DIS && filterControl < FILTER_OUT) {
				// when robot first gets cut off value for detection of an object, ignore them at first 
				filterControl++;
			} else if (distance < OBJECT_DIS) {
				// when robot keep getting the cut off value for detection of an object, record the distance and theata for localization of the target
				targets.add(new double[] {distance, nav.odometer.getAng()});
			} else {
				// when robot reads distance greater than cut off value again, reset filter to detect next target
				filterControl = 0;
			}
		}
		return this.getDestSet(targets);		//convert the distance and angle to x and y of targets to return 
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