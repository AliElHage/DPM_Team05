package CompetitionExecution;

public class TestAvoidance extends Thread{
	
	private static final int bandCenter=10, bandwidth=2, UsDis_OffWall=15, EscapeAngle= -70;				
	private static final int motorStraight = 200; // original motorStraight = 200 FILTER_OUT = 20
	private Navigation nav;
	private USPoller frontUS, rightUS;
	private double distance;
	private int pControl;
	private boolean handle;
	
	public TestAvoidance(Navigation nav, USPoller frontUS, USPoller rightUS) {
		this.nav = nav;
		this.frontUS = frontUS;
		this.rightUS = rightUS;
		this.handle = false;
	}
	
	public void run() {
		
		double startAngle = nav.odometer.getAng();     // suppose robot is facing the obstacle when avoidance is called
		
		nav.rotateLeft();
		frontUS.getRisingEdge(UsDis_OffWall, Searching.FILTER_OUT);
		nav.turn(-30);
		

		while(nav.odometer.getAng()- startAngle > EscapeAngle){
			distance = rightUS.readUSDistance();
			double error =  Math.abs(distance - bandCenter);
		
			pControl = 40 + (int)(error/bandwidth)*15;  //calculate the error after obtaining the distance
			if(pControl > 200) pControl = 200;
			
			
			if(distance > (bandCenter + bandwidth)){
				// if the robot is leaving the wall, then  turn left
				this.turnRight();
			}
			else if(distance < (bandCenter - bandwidth)){
				// if the robot is approaching the wall, then  turn right
				this.turnLeft();
			}
			else{
				// if the distance is within the error, then set motors back to move forward
				this.goStraight();
			}
		}
	
		this.handle = true;
	}
	
	private void turnLeft(){
		nav.setSpeeds(motorStraight,motorStraight+pControl);
	}
	
	private void turnRight(){
		nav.setSpeeds(motorStraight,motorStraight-pControl);
	}
	
	private void goStraight(){
		nav.setSpeeds(motorStraight, motorStraight);
	}
	
	public boolean handled(){
		return handle;
	}
	
}
