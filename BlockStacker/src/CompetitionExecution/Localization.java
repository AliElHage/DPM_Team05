package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

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
	private int ROTATION_SPEED = 150;
	private Odometer odo;
	private Navigation navigator;
	
	/**
	 * ultrasonic sensor localization specific variables
	 */
	private SampleProvider usSensor1, usSensor2;
	private float[] usData1, usData2;
	private int threshDist = 20;
	
	/**
	 * light sensor localization specific variables
	 */
	private static SampleProvider colorSensor;
	private static float[] colorData;	
	private int extraDist = 4;
	private double distance = 12.5;
	double x, y, angleX, angleY;
	
	/**
	 * constructor for Localizer that can do both light and ultrasonic sensor parts of localization, each having its own associated methods
	 * @param odo same odometer to be used for determining position in localize() and zeroRobot() methods
	 * @param usSensor1 right sensor used to localize robot in falling edge style
	 * @param usSensor2 left sensor used to localize robot in falling edge style
	 * @param usData used to hold data from us sensor
	 * @param colorSensor used to zero robot on line intersection using light sensor
	 * @param colorData used to hold data from light sensor
	 * @param leftMotor left motor used to physically move robot
	 * @param rightMotor right motor used to physically move robot
	 * @param navigator same navigator to be used for moving robot in localize() and zeroRobot() methods
	 */
	public Localization(Odometer odo,  SampleProvider usSensor1, SampleProvider usSensor2, float[] usData1,
			float[] usData2, SampleProvider colorSensor, float[] colorData, EV3LargeRegulatedMotor leftMotor, 
			EV3LargeRegulatedMotor rightMotor, Navigation navigator) {
		this.odo = odo;
		this.usSensor1 = usSensor1;
		this.usSensor2 = usSensor2;
		this.usData1 = usData1;
		this.usData2 = usData2;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
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
			
//			/** 
//			 * if the robot starts facing a wall, turn it 180 degrees so it has passed
//			 * the appointed threshold distance, then start rising edge procedure
//			 */
//			if (getFilteredData1() <= threshDist || getFilteredData2() <= threshDist) {
//				leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 180), true);
//				rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 180), false);
//			
//			}
			
			//start rising edge procedure by turning robot
			leftMotor.forward();
			rightMotor.backward();
			
			/** 
			 * continuously check left sensor for a wall while turning, if 
			 * one is found, beep, then stop the robot and record the angle 
			 * it is at in angleA, the first angle used for angular 
			 * positioning, then continue the procedure
			 */
			while (true) {
				if(getFilteredData1() <= threshDist){
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					if(odo.getAng() < 270) {
						angleA = 270 + odo.getAng();
					} else {
						angleA = odo.getAng() - 90;
					}
					break;
				}
			}
			
			LCD.drawString("AngleA: " + angleA, 0, 0);
			LCD.drawString("OdometerA: " + odo.getAng(), 0, 1);
			
//			/** 
//			 * stop checking for distance from wall while the robot turns 90 degrees 
//			 * to ensure that the sensor does not stop rotation too early
//			 */
//			leftMotor.rotate(-convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), true);
//			rightMotor.rotate(convertAngle(Main.WHEEL_RADIUS, Main.WIDTH, 90), false);

			/**
			 * start normal turning clockwise
			 */
			rightMotor.forward();
			leftMotor.backward();
			
			/** 
			 * continuously check right sensor for a wall while turning, if 
			 * one is found, beep, then stop the robot and record the angle 
			 * it is at in angleB, the second angle used for angular positioning
			 */
			while (true){
				if(getFilteredData2() <= threshDist){
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					angleB = 90 + odo.getAng();
					break;
				}
			}
			
			LCD.drawString("AngleB: " + angleB, 0, 2);
			LCD.drawString("OdometerB: " + odo.getAng(), 0, 3);
			
			/**
			 * calculations for average angle based on formulas given in slides
			 */
			if (angleA > angleB){
				angleAvg = 225 - (angleA + angleB)/2;
			}
			else{
				angleAvg =  45 - (angleA + angleB)/2;
			}
			
			LCD.drawString("Avg: " + angleAvg, 0, 4);
			LCD.drawString("Avg+B: " + (angleAvg+angleB), 0, 5);
			
			odo.setPosition(new double [] {0.0, 0.0, (angleAvg + angleB - 90)}, new boolean []{true, true, true});
			
			/**
			 * turn robot to face along the Y-axis
			 */
			navigator.turnTo(0, true);
	}
	
	/**
	 * polling method for right US sensor
	 * @return report distance of us1 from closest object
	 */
	public float getFilteredData1() {
		
		usSensor1.fetchSample(usData1, 0);
		float distance = 100*usData1[0];
				
		return distance;
	}
	
	/**
	 * polling method for left US sensor
	 * @return report distance of us2 from closest object
	 */
	public float getFilteredData2() {
		usSensor2.fetchSample(usData2, 0);
		float distance = 100*usData2[0];
				
		return distance;
	}

//	/**
//	 * methods originally from SquareDriver class in Lab2
//	 * @param radius
//	 * @param width
//	 * @param angle
//	 * @return
//	 */
//	private static int convertAngle(double radius, double width, double angle) {
//		return convertDistance(radius, Math.PI * width * angle / 360.0);
//	}
//	
//	/**
//	 * 
//	 * @param radius
//	 * @param distance
//	 * @return
//	 */
//	private static int convertDistance(double radius, double distance) {
//		return (int) ((180.0 * distance) / (Math.PI * radius));
//	}
	
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
		
//		/**
//		 * turns robot 180 degrees so that it can check lines by turning clockwise
//		 */
//		navigator.turnTo(180, true);
		
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
			if (lineCrossed()) {
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
		leftMotor.stop();
		rightMotor.stop();
		
//		LCD.drawString("First Angle: " + angles[0], 0, 0);
//		LCD.drawString("Second Angle: " + angles[1], 0, 1);
//		LCD.drawString("Third Angle: " + angles[2], 0, 2);
//		LCD.drawString("Fourth Angle: " + angles[3], 0, 3);
		
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
		
		/**
		 * turns to theta=0 to make it easier to set odometer position
		 */
//		navigator.turnTo(0, true);
		
		/**
		 * sets x and y knowing that theta is 0
		 */
		odo.setPosition(new double [] {x, y, odo.getAng()}, new boolean []{true, true, true});
		
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
	 * polls color sensor to get the data to see if it passes over a line.
	 * returns a boolean true if it does cross a line.
	 */
	private static boolean lineCrossed(){
		colorSensor.fetchSample(colorData, 0);
//		System.out.println("Sensor reading: " + colorData[0]);
		if(colorData[0] < 0.30){
			return true;
		}
		else{
			return false;
		}
	}
}