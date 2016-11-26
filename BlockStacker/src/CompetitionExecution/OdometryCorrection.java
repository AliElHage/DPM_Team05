package CompetitionExecution;

import javax.swing.text.NavigationFilter;

import lejos.utility.Delay;

public class OdometryCorrection extends Thread{
	
	private static final double sensorOffset = 13.5;
	final static double tileLength = 30.48;
	
	private boolean isVertical;
	private Navigation nav;
	private Odometer odometer;
	private LightPoller lightSensor;
	private double xDestination, yDestination, xPosition, yPosition, leftInitialTacho, rightInitialTacho,
	leftCurrentTacho, rightCurrentTacho, angle, actualAngle, distanceTraveled, computedX, computedY,
	actualX, actualY;
	private double[] position = new double[3], intersection;
	private int counter = 0;
	
	public OdometryCorrection(Navigation nav,  Odometer odometer, LightPoller lightSensor){
		
		this.odometer = odometer;
		this.lightSensor = lightSensor;
		
	}
	
	public void run(){
		
		Delay.msDelay(5000);
		
		while(true){
			
			// detect start of traveling
			while(true){
				
				if(nav.isTraveling()){
					
					// get current position
					xPosition = odometer.getX();
					yPosition = odometer.getY();
					
					// fetch arguments of travel function
					xDestination = nav.getDesiredX();
					yDestination = nav.getDesiredY();
					angle = nav.getDesiredAngle();	// create in Navigation class
					
					// get initial tacho count
					leftInitialTacho = odometer.getLeftMotor().getTachoCount();
					rightInitialTacho = odometer.getRightMotor().getTachoCount();
					
					break;
					
				}
				
			}
			
			while(nav.isTraveling()){
				
				if(lightSensor.lineCrossed()){
					
					leftCurrentTacho = odometer.getLeftMotor().getTachoCount();
					rightCurrentTacho = odometer.getRightMotor().getTachoCount();
					
					distanceTraveled = computeDistanceTraveled();
					leftInitialTacho = leftCurrentTacho;
					leftInitialTacho = leftCurrentTacho;
					
					intersection = findIntersection(xPosition, yPosition, angle);
					
					if(isVertical == true){
						
						computedY = intersection[1];
						actualY = Math.sqrt(Math.pow(distanceTraveled, 2) - Math.pow(Math.abs(intersection[0] - xPosition), 2));
						
						actualAngle = Math.atan(actualY / intersection[0]);
						
						odometer.setX(intersection[0] + sensorOffset * Math.cos(actualAngle));
						odometer.setY(intersection[1] + sensorOffset * Math.sin(actualAngle));
						odometer.setTheta(180 * actualAngle / Math.PI);
						
						xPosition = odometer.getX();
						yPosition = odometer.getY();
						leftInitialTacho = leftCurrentTacho;
						rightInitialTacho = leftCurrentTacho;
						
					}
					
					else if(isVertical == false){
						
						computedX = intersection[0];
						actualX = Math.sqrt(Math.pow(distanceTraveled, 2) - Math.pow(Math.abs(intersection[1] - yPosition), 2));
						
						actualAngle = Math.atan(intersection[1] / actualX);
						
						if(intersection[0] - xPosition > 0){
							
							odometer.setX(intersection[0] + sensorOffset * Math.tan(actualAngle));
							
						}
						else{
							
							odometer.setX(intersection[0] - sensorOffset * Math.tan(actualAngle));
							
						}
						
						if(intersection[1] - yPosition > 0){
							
							odometer.setY(intersection[1] + sensorOffset * Math.tan(actualAngle));
							
						}
						
						odometer.setY(intersection[1]);
						yPosition = intersection[1];
						odometer.setX(Math.sqrt(Math.pow(distanceTraveled, 2) - Math.pow(Math.abs(intersection[1] - yPosition), 2)));
						
						xPosition = odometer.getX();
						yPosition = odometer.getY();
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
	
	public double[] findIntersection(double xPosition, double yPosition, double angle){
			
		double[] intersection = new double[2];
		double xDistFromLine, yDistFromLine, xComponent, yComponent, originalX, originalY;
		int xCounter = 0, yCounter = 0, index = 0;
		
		originalX = xPosition;
		originalY = yPosition;
		
		while(xPosition > 30.48){
			xPosition -= 30.48;
			xCounter++;
		}
		
		while(xPosition < 0){
			xPosition += 30.48;
			xCounter++;
		}
		
		while(yPosition > 30.48){
			yPosition -= 30.48;
			yCounter++;
		}
		
		while(yPosition < 0){
			yPosition += 30.48;
			yCounter++;
		}
		
		if(angle >= 0 && angle < 90){
			xDistFromLine = 30.48 - xPosition;
			yDistFromLine = 30.48 - yPosition;
			yComponent = Math.abs(xDistFromLine * Math.tan(Math.PI / 180 * angle));
			xComponent = Math.abs(yDistFromLine / Math.tan(Math.PI / 180 * angle));
		}
		else if(angle >= 90 && angle < 180){
			xDistFromLine = xPosition;
			yDistFromLine = 30.48 - yPosition;
			yComponent = Math.abs(xDistFromLine * Math.tan(Math.PI / 180 * (180 - angle)));
			xComponent = Math.abs(yDistFromLine / Math.tan(Math.PI / 180 * angle));
		}
		else if(angle >= 180 && angle < 270){
			xDistFromLine = xPosition;
			yDistFromLine = yPosition;
			yComponent = Math.abs(xDistFromLine * Math.tan(Math.PI / 180 * (angle - 180)));
			xComponent = Math.abs(yDistFromLine / Math.tan(Math.PI / 180 * (angle - 180)));
			System.out.println(xComponent);
		}
		else{
			xDistFromLine = 30.48 - xPosition;
			yDistFromLine = yPosition;
			yComponent = Math.abs(xDistFromLine * Math.tan(Math.PI / 180 * angle));
			xComponent = Math.abs(yDistFromLine / Math.tan(Math.PI / 180 *((360 - angle))));
		}
		
		//System.out.println(xComponent);
		//System.out.println(yComponent);
		
		if(angle >= 0 && angle < 90){
			
			if(yComponent + yPosition > 30.48){
				System.out.println("Horizontal Line");
				isVertical = false;
				intersection[1] = 30.48 + yCounter * 30.48;
				intersection[0] = originalX + yDistFromLine * Math.tan(Math.PI / 180 * (90 - angle));
			}
			
			else{
				System.out.println("Vertical Line");
				isVertical = true;
				intersection[0] = 30.48 + xCounter * 30.48;
				intersection[1] = originalY + xDistFromLine * Math.tan(Math.PI / 180 * angle);
				
			}
			
		}
		
		else if(angle >= 90 && angle < 180){
			
			if(yComponent + yPosition > 30.48){
				System.out.println("Horizontal Line");
				isVertical = false;
				intersection[1] = 30.48 + yCounter * 30.48;
				intersection[0] = originalX - yDistFromLine * Math.tan(Math.PI / 180 * (90 - (180 - angle)));
				
			}
			
			else{
				
				System.out.println("Vertical Line");
				isVertical = true;
				intersection[0] = xCounter * 30.48;
				intersection[1] = originalY + xDistFromLine * Math.tan(Math.PI / 180 * (180 - angle));
				
			}
			
		}
		
		else if(angle >= 180 && angle < 270){
			
			if(xPosition - xComponent > 0){
				
				System.out.println("Horizontal Line");
				isVertical = false;
				intersection[1] = yCounter * 30.48;
				intersection[0] = originalX - yDistFromLine * Math.tan(Math.PI / 180 * (90 - (angle - 180)));
				
			}
			
			else{
				
				System.out.println("Vertical Line");
				isVertical = true;
				intersection[0] = xCounter * 30.48;
				intersection[1] = originalY - xDistFromLine * Math.tan(Math.PI / 180 * (angle - 180));
				
			}
			
		}
		
		else if(angle >= 270 && angle < 360){
			
			if(xComponent + xPosition < 30.48){
				System.out.println("Horizontal Line");
				isVertical = false;
				intersection[1] = yCounter * 30.48;
				intersection[0] = originalX + yDistFromLine * Math.tan(Math.PI / 180 * (90 -(360 - angle)));
			}
			
			else{
				
				System.out.println("Vertical Line");
				intersection[0] = 30.48 + xCounter * 30.48;
				intersection[1] = originalY - xDistFromLine * Math.tan(Math.PI / 180 * ((360 - angle)));
				
			}
			
		}
		
		return intersection;
	}
	
	private int countLines(double coordinate){
		
		int counter = 0;
		while(coordinate > tileLength){
			coordinate -= tileLength;
			counter++;
		}
		
		return counter;
		
	}
}
