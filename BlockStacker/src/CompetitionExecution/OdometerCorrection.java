package CompetitionExecution;

public class OdometerCorrection {
	
	private static final double sensorOffset = 5;
	
	private Odometer odometer;
	private Navigation navigation;
	private LightPoller lightSensor;
	private double xDestination, yDestination, xPosition, yPosition, leftInitialTacho, rightInitialTacho,
	leftCurrentTacho, rightCurrentTacho, angle, distanceTraveled;
	private double[] position = new double[3], intersection;
	
	public OdometerCorrection(Odometer odometer, LightPoller lightSensor){
		
		this.odometer = odometer;
		this.lightSensor = lightSensor;
		
	}
	
	public void run(){
		
		while(true){
			
			// detect start of traveling
			while(true){
				
				if(navigation.isTraveling()){
					
					// fetch arguments of travel function
					xDestination = navigation.getDesiredX();
					yDestination = navigation.getDesiredY();
					angle = Navigation.getAngle();	// create in Navigation class
					
					// get initial tacho count
					leftInitialTacho = odometer.getLeftMotor().getTachoCount();
					rightInitialTacho = odometer.getRightMotor().getTachoCount();
					
					// get current position
					xPosition = odometer.getX();
					yPosition = odometer.getY();
					
					break;
					
				}
				
			}
			
			while(navigation.isTraveling()){
				
				if(lightSensor.lineDetected()){
					
					leftCurrentTacho = odometer.getLeftMotor().getTachoCount();
					rightCurrentTacho = odometer.getRightMotor().getTachoCount();
					
					distanceTraveled = computeDistanceTraveled();
					leftInitialTacho = leftCurrentTacho;
					leftInitialTacho = leftCurrentTacho;
					
					intersection = Test.findIntersection(xPosition, yPosition, angle);
					
					if(Test.isVertical){
						
						odometer.setX(intersection[0]);
						xPosition = intersection[0];
						odometer.setY(Math.sqrt(Math.pow(distanceTraveled, 2) - Math.pow(Math.abs(intersection[0] - xPosition), 2)));
						yPosition = odometer.getY();
						leftInitialTacho = leftCurrentTacho;
						rightInitialTacho = leftCurrentTacho;
						
					}
					
					else if(!Test.isVertical){
						
						odometer.setY(intersection[1]);
						yPosition = intersection[1];
						odometer.setX(Math.sqrt(Math.pow(distanceTraveled, 2) - Math.pow(Math.abs(intersection[1] - yPosition), 2)));
						xPosition = odometer.getX();
						leftInitialTacho = leftCurrentTacho;
						rightInitialTacho = leftCurrentTacho;
						
					}
				}
				
			}
			
		}
		
	}
	
	private double computeDistanceTraveled(){
		
		double traveledDistance = 0;
		
		traveledDistance = ((leftCurrentTacho - leftInitialTacho) + (rightCurrentTacho - rightInitialTacho)) / 2;
		
		return traveledDistance;
		
	}
	
}
