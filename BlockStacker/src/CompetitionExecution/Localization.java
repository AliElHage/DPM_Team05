package CompetitionExecution;

import lejos.hardware.Sound;
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
	private int FORWARD_SPEED = 200;
	private int ROTATION_SPEED = 150;		//150
	private int LEFT_TURN_SPEED = 75;
	private Odometer odo;
	private Navigation navigator;
	boolean leftTurnToZero;
	
	/**
	 * ultrasonic sensor localization specific variables
	 */
	private USPoller rightUS, leftUS, frontUS;
	private int threshDist = 20;
	private int wallThresh = 40;
	private int correctionAngRight = 15;
	private int correctionAngLeft = 15;
	
	/**
	 * light sensor localization specific variables
	 */
	private static LightPoller lightSensor;
	private int extraDist = 4;
	private double distance = 12.5;
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
	//USPoller rightUS, USPoller leftUS, 
	
	public Localization(Odometer odo,  
			USPoller frontUS,
			LightPoller lightSensor, EV3LargeRegulatedMotor leftMotor, 
			EV3LargeRegulatedMotor rightMotor, Navigation navigator) {
		this.odo = odo;
//		this.rightUS = rightUS;
//		this.leftUS = leftUS;
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
		
			
		/* if the robot starts facing a wall, turn it 180 degrees so it has passed
		 * the appointed threshold distance, then start rising edge procedure
		 */
		if (frontUS.readUSDistance() <= threshDist) {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.setSpeed(ROTATION_SPEED);
			
			/* if the robot starts facing a wall, turn it until it has passed
			 * the appointed threshold distance of 35cm, then start falling edge procedure
			 */			
			if(frontUS.readUSDistance() >= threshDist){
				//turn the robot clockwise
				leftMotor.forward();
				rightMotor.backward();
				
				while (true){
					if(frontUS.readUSDistance() <= threshDist){
						leftMotor.stop();
						rightMotor.stop();
						break;
					}
				}
			}
			
			/* stop checking for distance from wall while the robot turns 45 degrees 
			 * to ensure that the sensor does not stop rotation too early
			 */
			leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), false);
			
			//start normal turning clockwise
			leftMotor.forward();
			rightMotor.backward();
			
			/* continuously check for a distance farther than the threshold of
			 * 35cm while turning, if one is found, beep, then stop the robot
			 * and record the angle it is at in angleA, the first angle used for
			 * angular positioning
			 */
			while (true){
				if(frontUS.readUSDistance() > threshDist){
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					angleA = odo.getAng();
					break;
				}
			}
			
			/* Turn toward the wall a bit to ensure the sensor detects the wall
			 * properly and does not accidentally detect it is farther than the
			 * threshold
			 */
			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), true);
			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 45), false);
			
			//start normal turning counterclockwise
			leftMotor.backward();
			rightMotor.forward();
			
			/* continuously check for a distance farther than the threshold of
			 * 35cm while turning, if one is found, beep, then stop the robot
			 * and record the angle it is at in angleB, the second angle used for
			 * angular positioning
			 */			
			while (true){
				if(frontUS.readUSDistance() > threshDist){
					Sound.beep();
					rightMotor.stop();
					leftMotor.stop();
					angleB = odo.getAng();
					break;
				}
			}
			
			//calculations for average angle based on formulas given in slides
			if(angleA > angleB){
				angleAvg = 225 - (angleA + angleB)/2;
			}
			else{
				angleAvg =  45 - (angleA + angleB)/2;
			}
			double ang = angleA+angleAvg;
			LCD.drawString("Pos: " + ang , 0, 6);
			
			odo.setPosition(new double [] {0.0, 0.0, angleA + angleAvg}, new boolean []{true, true, true});
			
			//turn robot to face along the X-axis
			navigator.turnTo(180, true);
			
		
		} else {
		
			//start rising edge procedure by turning robot
			leftMotor.forward();
			rightMotor.backward();
			
			/* continuously check for a wall while turning, if one is found, 
			 * beep, then stop the robot and record the angle it is at in 
			 * angleA, the first angle used for angular positioning, then 
			 * continue the procedure
			 */
			while (true) {
				if(frontUS.readUSDistance() <= threshDist){
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					angleA = odo.getAng();
					break;
				}
			}
			
			
			/* stop checking for distance from wall while the robot turns 90 degrees 
			 * to ensure that the sensor does not stop rotation too early
			 */
			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), true);
			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), false);
	
			//start normal turning clockwise
			rightMotor.forward();
			leftMotor.backward();
			
			/* continuously check for a wall while turning, if one is found, beep,
			 * then stop the robot and record the angle it is at in angleB,
			 * the second angle used for angular positioning
			 */
			while (true){
				if(frontUS.readUSDistance() <= threshDist){
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					angleB = odo.getAng();
					break;
				}
			}
			
			//calculations for average angle based on formulas given in slides
			if (angleA > angleB){
				angleAvg = 225 - (angleA + angleB)/2;
			}
			else{
				angleAvg =  45 - (angleA + angleB)/2;
			}
			
			odo.setPosition(new double [] {0.0, 0.0, angleB + angleAvg}, new boolean []{true, true, true});
			
			//turn robot to face along the X-axis
			navigator.turnTo(0, true);
		}
		
		
		
		
//		double angleA, angleB, angleAvg;
//		
//		leftMotor.setSpeed(ROTATION_SPEED);
//		rightMotor.setSpeed(ROTATION_SPEED);
//			
//			Delay.msDelay(200);
//			
//			/** 
//			 * if the robot starts facing a wall, turn it 180 degrees so it has passed
//			 * the appointed threshold distance, then start rising edge procedure
//			 */
//			if (frontUS.readUSDistance() <= wallThresh) {
//				leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 180), true);
//				rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 180), false);
//			
//			}
//			
//			/**
//			 * start turning in direction in which side sensor is farther from the wall
//			 */
//			if(rightUS.readUSDistance() > leftUS.readUSDistance()) {
//				
//				leftTurnToZero = false;
//				
//				/**
//				 * start rising edge procedure by turning robot
//				 */
//				leftMotor.forward();
//				rightMotor.backward();
//				
//				/** 
//				 * continuously check left sensor for a wall while turning, if 
//				 * one is found, beep, then stop the robot and record the angle 
//				 * it is at in angleA, the first angle used for angular 
//				 * positioning, then continue the procedure
//				 */
//				while (true) {
//					if(rightUS.readUSDistance() <= threshDist){
//						Sound.beep();
//						navigator.stopMoving();
//						if(odo.getAng() < 270) {
//							angleA = 270 + odo.getAng();
//						} else {
//							angleA = odo.getAng() - 90;
//						}
//						break;
//					}
//				}
//	
//				navigator.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
//				
//				/**
//				 * start normal turning clockwise
//				 */
//				rightMotor.forward();
//				leftMotor.backward();
//				
//				/** 
//				 * continuously check right sensor for a wall while turning, if 
//				 * one is found, beep, then stop the robot and record the angle 
//				 * it is at in angleB, the second angle used for angular positioning
//				 */
//				while (true){
//					if(leftUS.readUSDistance() <= threshDist){
//						Sound.beep();
//						navigator.stopMoving();
//						angleB = 90 + odo.getAng();
//						break;
//					}
//				}	
//			} else {
//				
//				leftTurnToZero = true;
//				
//				/**
//				 * start rising edge procedure by turning robot
//				 */
//				rightMotor.forward();
//				leftMotor.backward();
//				
//				/** 
//				 * continuously check right sensor for a wall while turning, if 
//				 * one is found, beep, then stop the robot and record the angle 
//				 * it is at in angleA, the first angle used for angular positioning
//				 */
//				while (true) {
//					if(leftUS.readUSDistance() <= threshDist){
//						Sound.beep();
//						navigator.stopMoving();
//						if(odo.getAng() < 270) {
//							angleA = 270 + odo.getAng();
//						} else {
//							angleA = odo.getAng() - 90;
//						}
//						break;
//					}
//				}
//				
//				navigator.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
//				
//				/**
//				 * start normal turning couterclockwise 
//				 */
//				leftMotor.forward();
//				rightMotor.backward();
//				
//				/** 
//				 * continuously check left sensor for a wall while turning, if 
//				 * one is found, beep, then stop the robot and record the angle 
//				 * it is at in angleB, the second angle used for angular positioning
//				 */
//				while (true){
//					if(rightUS.readUSDistance() <= threshDist){
//						Sound.beep();
//						navigator.stopMoving();
//						angleB = 90 + odo.getAng();
//						break;
//					}
//				}
//			}
//			
//			/**
//			 * calculations for average angle based on formulas given in slides
//			 */
//			if (angleA > angleB){
//				angleAvg = 225 - (angleA + angleB)/2;
//			}
//			else{
//				angleAvg =  45 - (angleA + angleB)/2;
//			}
//			
//			LCD.drawString("Avg: " + angleAvg, 0, 4);
//			LCD.drawString("Avg+B: " + (angleAvg+angleB), 0, 5);
//			
//			if(leftTurnToZero) {
//				odo.setPosition(new double [] {0.0, 0.0, (angleAvg + angleB - 90 + correctionAngLeft)}, new boolean []{true, true, true});
//				navigator.setSpeeds(LEFT_TURN_SPEED, LEFT_TURN_SPEED);
//			} else {
//				odo.setPosition(new double [] {0.0, 0.0, (angleAvg + angleB - 90 + correctionAngRight)}, new boolean []{true, true, true});
//				navigator.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
//			}
//			
//			/**
//			 * turn robot to face along the Y-axis
//			 */
//			navigator.turnTo(0, true);
	}

	/**
	 * determine where the zero-zero point on the floor grid is by getting angles between
	 * the first and third and second and fourth grid lines the light sensor detects and
	 * moves robot to this location to manually set odometer to 0-0-0.
	 */
	public void zeroRobot() {
		LCD.clear();
		/**
		 * turns robot to 45 degrees, which can happen because the angle is already oriented
		 */
		navigator.turnTo(45, true);
		
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
				Sound.beep();
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
		
		if (leftTurnToZero) {
			y = (distance)*Math.cos(Math.toRadians(angleX));
		} else {
			y = (-distance)*Math.cos(Math.toRadians(angleX));
		}
		
		
		/**
		 * turns to theta=0 to make it easier to set odometer position
		 */
		
		/**
		 * sets x and y knowing that theta is 0
		 */
		odo.setPosition(new double [] {x, y, odo.getAng()}, new boolean []{true, true, true});
		
		navigator.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
		
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
		
		/**
		 * based on if our team is builder or collector, BSC or CSC will hold our starting corner, respectively
		 * set x,y,theta based on starting corner
		 */
		if (Main.BTN == 5) {
			if (Main.BSC == 1) {
				/**
				 * set odometer to confirm that it is now at 0,0,0
				 */
				odo.setPosition(new double [] {0, 0, odo.getAng()}, new boolean [] {true, true, true});
			} else if (Main.BSC == 2) {
				/**
				 * set odometer to confirm that it is now at 10,0,90
				 */
				odo.setPosition(new double [] {10, 0, odo.getAng()+90}, new boolean [] {true, true, true});
			} else if (Main.BSC == 3) {
				/**
				 * set odometer to confirm that it is now at 10,10,180
				 */
				odo.setPosition(new double [] {10, 10, odo.getAng()+180}, new boolean [] {true, true, true});
			} else if (Main.BSC == 4) {
				/**
				 * set odometer to confirm that it is now at 0,10,270
				 */
				odo.setPosition(new double [] {0, 10, odo.getAng()+270}, new boolean [] {true, true, true});
			}
		} else if (Main.CTN == 5) {
			if (Main.CSC == 1) {
				/**
				 * set odometer to confirm that it is now at 0,0,0
				 */
				odo.setPosition(new double [] {0, 0, odo.getAng()}, new boolean [] {true, true, true});
			} else if (Main.CSC == 2) {
				/**
				 * set odometer to confirm that it is now at 10,0,90
				 */
				odo.setPosition(new double [] {10, 0, odo.getAng()+90}, new boolean [] {true, true, true});
			} else if (Main.CSC == 3) {
				/**
				 * set odometer to confirm that it is now at 10,10,180
				 */
				odo.setPosition(new double [] {10, 10, odo.getAng()+180}, new boolean [] {true, true, true});
			} else if (Main.CSC == 4) {
				/**
				 * set odometer to confirm that it is now at 0,10,270
				 */
				odo.setPosition(new double [] {0, 10, odo.getAng()+270}, new boolean [] {true, true, true});
			}
		}
		
		
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