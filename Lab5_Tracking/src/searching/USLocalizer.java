package searching;

import lejos.robotics.SampleProvider;

public class USLocalizer {
	
	final static double GRID_LEN = 25;  //28
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private Navigation navigator;
	
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, Navigation navigator) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.navigator = navigator;

	}
	
	public void doLocalization() {
		double angleA, angleB, angleDiff, turnAngle, disBack;
			
			// rotate the robot until it sees no wall
			navigator.rotateLeft();
			
			while(true){
				if((int)getFilteredData() > 30){
					//System.out.println("     NOW");
					break;
				}
			}
			
			// keep rotating until the robot sees a wall, then latch the angle
			
			while(true){
				if((int)getFilteredData() < 30){
					//System.out.println("WORK");
					angleA = odo.getAng();
					navigator.setSpeeds(0, 0);
					break;
				}
			}
			
			// switch direction and wait until it sees no wall
			
			navigator.rotateRight();
			
			while(true){
				if((int)getFilteredData() > 30){
					break;
				}
			}
			
			// keep rotating until the robot sees a wall, then latch the angle
			
			while(true){
				if((int)getFilteredData() < 30){
					angleB = odo.getAng();
					navigator.setSpeeds(0, 0);
					break;
				}
			}
			
			if(angleA > angleB){
				angleDiff = 225 - (angleA + angleB) / 2;
			}
			else{
				angleDiff = 45 - (angleA + angleB) / 2;
			}
			
			// wrap angle to positive value
			if(angleDiff < 0) angleDiff += 360;
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, angleDiff}, new boolean [] {true, true, true});
			
			// adjust angle
			turnAngle = angleDiff + 60;
			
			// wrap angle to less than 360
			if(turnAngle > 360) turnAngle -= 360;
			
			// rotate
			if(turnAngle < 0) navigator.turnTo(turnAngle + 360, true);
			else navigator.turnTo(turnAngle, true);
			
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true}); 		//reset odometer after calibration 
			
			navigator.turnTo(270, true);
			disBack = getFilteredData();			//get the distance to the back wall
			navigator.travelTo((GRID_LEN - disBack), (GRID_LEN - disBack));
			navigator.turnTo(0, true);
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true}); 		//reset odometer after calibration 
			
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0];
		distance *= 100;
		if(distance > 255) distance = 255;
		
		return distance;
	}

}
