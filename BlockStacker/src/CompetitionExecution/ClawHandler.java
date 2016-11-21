package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ClawHandler {
	
	final static int clawSpeed = 100;					// speed of clawMotor
	final static int pulleySpeed = 75;					// speed of pulleyMotor
	final static double initialHeight =10.5;			// initial position of claws relative to the ground
	final static double minDistanceFromGround = 2.2;	// minimum distance of claw relative to ground
	final static double safeDropDistance = 4.5;			// amount to lower the claws to safely drop blocks on top of each other
	final static double motorRadius = 0.7;				// distance between the middle of the motor and the peripheral holes
	final static int clawOpenAngle = 50, minorOpenAngle = 60, clawCloseAngle = 65;
	final static double offsetLift = 0.3; 
	private double angleToRelease, angleToLift, angleToSet;	// amount by which to move the claws from their initial position for pulley	
	private EV3LargeRegulatedMotor pulleyMotor, clawMotor;	// all claw-related motors
	private int counter;									// counts the amount of blocks stacked
	private static boolean isInitialized = false;			// true if claw has been initialized
	
	// NB FOR PULLEYSPEED: a negative speed is for lowering the claw; positive otherwise
	// NB FOR CLAWSOPENANGLE: a negative angle is for grabbing; releasing otherwise
	
	/** Constructor. */
	public ClawHandler(EV3LargeRegulatedMotor clawMotor, EV3LargeRegulatedMotor pulleyMotor){
		this.clawMotor = clawMotor;
		this.pulleyMotor = pulleyMotor;
		this.counter=0;
		this.angleToRelease = this.computePulleyTurnAngle(initialHeight -minDistanceFromGround);
		this.angleToLift = this.computePulleyTurnAngle(initialHeight - minDistanceFromGround + offsetLift);
		this.angleToSet = this.computePulleyTurnAngle(safeDropDistance);
		this.pulleyMotor.setSpeed(pulleySpeed);
		this.clawMotor.setSpeed(clawSpeed);
	}
	
	/** Initializes the claws. First method that should be called when using the claws. To be used
	 *  effectively, the claws should start at the top and be in a closed position,*/
	public void initializeClaw(){
		
		double angle;											// angle needed to lower claw to ground
		
		// set negative speed to lower the claw
		pulleyMotor.setSpeed(-pulleySpeed);
		
		isInitialized = true;
	}
	
	/**
	 * open the claw when it is fully closed
	 */
	public void open(){
		clawMotor.rotate((int)clawOpenAngle, false);
	}
	
	/**
	 * open the claw when it is grasping an object
	 */
	public void minorOpen(){
		clawMotor.rotate((int)minorOpenAngle, false);
	}
	
	/**
	 * close the claw
	 */
	public void close(){
		clawMotor.rotate(-(int)clawCloseAngle, false);
	}
	
	/**
	 * release the pulley down all the way down to the bottom  
	 */
	public void putDown(){
		pulleyMotor.rotate((int)angleToRelease, false);
	}
	
	/**
	 * release the pulley down to the top of the foam block
	 */
	public void putDownToObj(){
		pulleyMotor.rotate((int)angleToSet, false);
	}
	
	/**
	 * release the pulley down to bottom from the height of the foam block 
	 */
	public void putDownToBot(){
		double angleToGrab = computePulleyTurnAngle(initialHeight - minDistanceFromGround - safeDropDistance);
		pulleyMotor.rotate((int)angleToGrab, false);
	}
	
	/**
	 * pull pulley all the way up the top 
	 */
	public void pullUp(){
		pulleyMotor.rotate(-(int)angleToLift, false);
	}
	
	/** Release the claw and grasp the foam and pull up the claw. 
	 *	 Only call when the robot is at the appropriate distance.
	 *  (block should be below the claws before this method is called).*/
	public void grasp(){
		
		// if the robot holds no block
		if(counter == 0){
			//open the claw
			this.open();
			
			//release the claw down
			this.putDown();
			
			// grab block
			this.close();
			
			// lifts the block up
			this.pullUp();
			
			// count block
			counter++;
			return;
		}
		
		// gently place the held block on top of the other block
		this.putDownToObj();
		
		// release block from claws and lower the claw to ground block level
		this.minorOpen();
		this.putDownToBot();
		
		// grab ground block
		this.close();
		
		// lift blocks up
		this.pullUp();
		counter++;
	}
	
	/** Safely releases the tower on the ground.<br />
	 *  [WARNING] only call when robot is in the safe zone. */
	public void releaseTower(){
		
		// lower tower to ground
		this.putDown();
		
		// release tower
		this.minorOpen();
		this.counter = 0;
		Sound.beep();
		this.pullUp();
	}
	
	
	/** computes angle by which the pulleyMotor should rotate to lift the claws by distanceToRotate */
	public double computePulleyTurnAngle(double distanceToRotate){
		
		// compute the circumference described by a full turn of the motor
		double motorCircumference = 2 * Math.PI * motorRadius;
		
		// compute the angle required to turn in order to lift the claw by the desired distance
		double angle = distanceToRotate / motorCircumference * 360;
		
		return angle;
	}
	
	
	/** Gets the amount of blocks held by the robot. */
	public int getCounter(){
		return this.counter;
	}
	
}
