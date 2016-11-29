package CompetitionExecution;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * identifies (0,0) on grid and centralizes robot at this point
 * @author courtneywright
 *
 */
public class Localization {
	
	/**
	 * general variables used for both ultrasonic and light sensor parts of localization
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int FORWARD_SPEED = 300;
	private int ROTATION_SPEED = 250;		//150
	private int LEFT_TURN_SPEED = 75;
	private Odometer odo;
	private Navigation navigator;
	boolean leftTurnToZero;
	
	/**
	 * ultrasonic sensor localization specific variables
	 */
	private USPoller rightUS, leftUS, frontUS;
	private int threshDist = 20;
	private int wallThresh = 35;
	private boolean rising;
	
	/**
	 * light sensor localization specific variables
	 */
	private static LightPoller lightSensor;
	private double distance = 12.5;
	private double correctionAng = 90;
	double x, y, angleX, angleY;
	
	/**
	 * constructor for Localizer that can do both light and ultrasonic sensor parts of localization, each having its own associated methods
	 * @param odo same odometer to be used for determining position in localize() and zeroRobot() methods
	 * @param rightUS right sensor used to localize robot in falling edge style
	 * @param leftUS left sensor used to localize robot in falling edge style
	 * @param frontUS front sensor used to check if robot is facing wall
	 * @param lightSensor used to zero robot on line intersection using light sensor
	 * @param leftMotor left motor used to physically move robot
	 * @param rightMotor right motor used to physically move robot
	 * @param navigator same navigator to be used for moving robot in localize() and zeroRobot() methods
	 */
	
	public Localization(Odometer odo,  
			USPoller frontUS,
			LightPoller lightSensor, EV3LargeRegulatedMotor leftMotor, 
			EV3LargeRegulatedMotor rightMotor, Navigation navigator) {
		this.odo = odo;
		this.frontUS = frontUS;
		this.lightSensor = lightSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.navigator = navigator;
	}
	
	/**
	 * determine where the theta=0 is for robot placed anywhere within the starting square by checking the angles
	 * the robot much turn for the US sensor to be within the threshold distance from each of the walls
	 */
	public void localize() {
		
		double angleA, angleB, angleAvg;
		
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		if (frontUS.readUSDistance() <= wallThresh) {
			
			rising = true;
			
//			navigator.turn(180, 200);
			
			/**
			 * The robot should look for the "rising edges:" the points where it no 
			 * longer sees the wall.
			 */
			
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.setSpeed(ROTATION_SPEED);
			
//			/**
//			 * Stops checking for distance from wall while the robot turns 45 degrees 
//			 * to ensure that the sensor does not stop rotation too early
//			 */
//			leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), true);
//			rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), false);
			
			/**
			 * Starts normal turning clockwise
			 */
			leftMotor.forward();
			rightMotor.backward();
			
			/**
			 * Continuously checks for a distance farther than the threshold of
			 * 35cm while turning, if one is found, beep, then stop the robot
			 * and record the angle it is at in angleA, the first angle used for
			 * angular positioning
			 */
			while (true){
//				if(frontUS.readUSDistance() > threshDist) {
				if(frontUS.getRisingEdge(threshDist, 10) > threshDist) {
					Delay.msDelay(250);
					navigator.stopMoving();
//					leftMotor.stop();
//					rightMotor.stop();
					angleA = odo.getAng();
					break;
				}
			}
			
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			
			/**
			 * Turn toward the wall a bit to ensure the sensor detects the wall
			 * properly and does not accidentally detect it is farther than the
			 * threshold
			 */
//			navigator.turn(20);
//			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), true);
//			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), false);
			
			/**
			 * Starts normal turning counterclockwise
			 */
			leftMotor.backward();
			rightMotor.forward();
			
			Delay.msDelay(1250);
			
			/**
			 * Continuously checks for a distance farther than the threshold
			 * while turning, if one is found, beep, then stop the robot
			 * and record the angle it is at in angleB, the second angle used for
			 * angular positioning
			 */			
			while (true){
//				if(frontUS.readUSDistance() > threshDist){
				if(frontUS.getRisingEdge(threshDist, 10) > threshDist) {
					Delay.msDelay(250);
					navigator.stopMoving();
//					rightMotor.stop();
//					leftMotor.stop();
					angleB = odo.getAng();
					break;
				}
			}
			
			/**
			 * Calculations for average angle based on formulas given in slides
			 */
			if(angleA > angleB){
				angleAvg = 225 - (angleA + angleB)/2;
			}
			else{
				angleAvg =  45 - (angleA + angleB)/2;
			}
			
			double ang = angleA+angleAvg;
			LCD.drawString("Pos: " + ang , 0, 6);
			
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			
			odo.setPosition(new double [] {0.0, 0.0, angleA + angleAvg + correctionAng}, new boolean []{true, true, true});
			
			/**
			 * Turns robot to face along the X-axis
			 */
			navigator.turnTo(0, true);
			
		
		} else {
		
			rising = false;
			
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			/**
			 * Starts rising edge procedure by turning robot
			 */
			leftMotor.forward();
			rightMotor.backward();
			
			/**
			 * Continuously checks for a wall while turning, if one is found, 
			 * beep, then stop the robot and record the angle it is at in 
			 * angleA, the first angle used for angular positioning, then 
			 * continue the procedure
			 */
			while (true) {
				if(frontUS.readUSDistance() <= threshDist){
//					Sound.beep();
					navigator.stopMoving();
//					leftMotor.stop();
//					rightMotor.stop();
					angleA = odo.getAng();
					break;
				}
			}
			
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			
//			/**
//			 * Stops checking for distance from wall while the robot turns 90 degrees 
//			 * to ensure that the sensor does not stop rotation too early
//			 */
//			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), true);
//			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), false);
	
			/**
			 * Starts normal turning clockwise
			 */
			rightMotor.forward();
			leftMotor.backward();
			
			Delay.msDelay(1250);
			
			/**
			 * Continuously checks for a wall while turning, if one is found,stop the 
			 * robot and record the angle it is at in angleB,
			 * the second angle used for angular positioning
			 */
			while (true){
				if(frontUS.readUSDistance() <= threshDist){
//					Sound.beep();
					navigator.stopMoving();
//					leftMotor.stop();
//					rightMotor.stop();
					angleB = odo.getAng();
					break;
				}
			}
			
			/**
			 * Calculations for average angle based on formulas given in slides
			 */
			if (angleA > angleB){
				angleAvg = 225 - (angleA + angleB)/2;
			}
			else{
				angleAvg =  45 - (angleA + angleB)/2;
			}
			
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			
			odo.setPosition(new double [] {0.0, 0.0, angleB + angleAvg}, new boolean []{true, true, true});
			
			/**
			 * Turns robot to face along the X-axis
			 */
//			navigator.turnTo(0, true);
		}
		
	}

	/**
	 * determine where the zero-zero point on the floor grid is by getting angles between
	 * the first and third and second and fourth grid lines the light sensor detects and
	 * moves robot to this location to manually set odometer to 0-0-0.
	 */
	public void zeroRobot() {
		/*LCD.clear();*/
		/**
		 * turns robot to 45 degrees, which can happen because the angle is already oriented
		 */
		navigator.turnTo(45, true, ROTATION_SPEED);
//		navigator.turnTo(45, true);
		
		/**
		 * moves robot forward 15cm to properly place sensor to check lines
		 */
		navigator.goForward(15);
		
		/**
		 * instantiate array that will hold angle at each line and index of each line
		 */
		double angles[] = new double[4];
		int lineNumber = 0;
		
		/**
		 * sets rotation speed
		 */
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		/**
		 * starts turning to check lines
		 */
		leftMotor.forward();
		rightMotor.backward();
		
		/**
		 * polls for data by checking lineCrossed() method below this one. If a line is encountered,
		 * robot beeps and stores angle it is at in angles array at position of line (line 0, then line
		 * 1, then line 2, then line 3). Then sleeps for 500ms so that the line is not detected twice.
		 */
		while (lineNumber < 4) {
			if (lightSensor.lineCrossed()) {
				angles[lineNumber] = odo.getAng();
				lineNumber +=1;
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} 
			
		}
		
		/**
		 * stops both motors
		 */
		navigator.stopMoving();

		/**
		 * formula from the slides, angles are those taken when passing line
		 */
		angleY = (angles[3] - angles[1])/2;
		angleX = (angles[2] - angles[0])/2;
		
		/**
		 * formula came from slides, however it was determined experimentally that there was a mistake in the
		 * calculation for x and it was returned as the absolute value of what it should have been when calculated
		 * with a negative, so the formula was changed to exclude the negative
		 */
		x = (-distance)*Math.cos(Math.toRadians(angleY));
		y = (distance)*Math.cos(Math.toRadians(angleX));
//		if (rising) {
//			y = (-distance)*Math.cos(Math.toRadians(angleX));
//		}
//		else {
//			y = (distance)*Math.cos(Math.toRadians(angleX));
//		}
		
		/**
		 * turns to theta=0 to make it easier to set odometer position
		 */
		
		/**
		 * sets x and y knowing that theta is 0
		 */
		odo.setPosition(new double [] {x, y, odo.getAng()}, new boolean []{true, true, true});
		
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		/**
		 * navigation method to move the robot to its final position based on calculation
		 */
		navigator.travelTo(0,0);
		
		/**
		 * stops the robot before it moves to its final position
		 */
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		/**
		 * navigation method to turn the robot to its final position based on calculation
		 */
		navigator.turnTo(0, true);
		
	}
	
	/**
	 * methods originally from SquareDriver class in Lab2
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}