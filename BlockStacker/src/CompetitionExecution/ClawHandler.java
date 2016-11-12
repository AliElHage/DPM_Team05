package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ClawHandler {
	
	final static int clawSpeed = 100;							// speed of clawMotor
	final static int pulleySpeed = 75;							// speed of pulleyMotor
	final static double initialHeight = 20.0;					// initial position of claws relative to the ground
	final static double minDistanceFromGround = 2.2;			// minimum distance of claw relative to ground
	final static double safeDropDistance = 1.0;					// amount to lower the claws to safely drop blocks on top of each other
	final static double safeLiftDistance = 5.0;					// amount by which the claw will be lifted each time it grabs a block
	final static double motorRadius = 0.7;						// distance between the middle of the motor and the peripheral holes
	final static double clawOpenAngle = 30;
	private static boolean isInitialized = false;				// true if claw has been initialized
	private EV3LargeRegulatedMotor pulleyMotor, clawMotor;		// all claw-related motors
	private int counter = 0;									// counts the amount of blocks stacked
	
	// NB FOR PULLEYSPEED: a negative speed is for lowering the claw; positive otherwise
	// NB FOR CLAWSOPENANGLE: a negative angle is for grabbing; releasing otherwise
	
	/** Constructor. */
	public ClawHandler(){}
	
	/** Initializes the claws. First method that should be called when using the claws. To be used
	 *  effectively, the claws should start at the top and be in a closed position,*/
	public void initializeClaw(){
		
		double angle;											// angle needed to lower claw to ground
		double descent = initialHeight - minDistanceFromGround;	// amount by which to lower the claws from their initial position
		
		// set negative speed to lower the claw
		pulleyMotor.setSpeed(-pulleySpeed);
		
		// compute turn angle to lower the claws to block level
		angle = computePulleyTurnAngle(descent);
		
		// lower claws to ground
		pulleyMotor.rotate((int)angle, false);
		
		// reset speed
		pulleyMotor.setSpeed(pulleySpeed);
		
		// set clawMotor speed and open claws
		clawMotor.setSpeed(clawSpeed);
		clawMotor.rotate((int)clawOpenAngle, false);
		
		isInitialized = true;
	}
	
	/** Lifts block. Only call when the robot is at the appropriate distance
	 *  (block should be between the claws before this method is called).*/
	public void lift(){
		
		// safety measure
		if(isInitialized == false){
			Sound.beep();
			System.out.println("FATAL ERROR: Claw has not been initialized.");
			return;
		}
		
		// angles by which to rotate to lift tower 5 cm, gently drop block and grab lowest block respectively
		double angleToLift = computePulleyTurnAngle(safeLiftDistance);
		double safeLoweringAngle = computePulleyTurnAngle(safeDropDistance);
		double angleToGrab = computePulleyTurnAngle(safeLiftDistance - safeDropDistance);
		
		// if the robot holds no block
		if(counter == 0){
			// grab block
			clawMotor.rotate(-(int)clawOpenAngle, false);
			
			// lifts the block 5 cm
			pulleyMotor.rotate((int)angleToLift, false);
			
			// count block
			counter++;
			return;
		}
		
		// gently place the held block on top of the other block
		pulleyMotor.setSpeed(-pulleySpeed);
		pulleyMotor.rotate((int)safeLoweringAngle, false);
		
		// release block from claws and lower the claw to ground block level
		clawMotor.rotate((int)clawOpenAngle, false);
		pulleyMotor.rotate((int)angleToGrab, false);
		
		// grab ground block
		clawMotor.rotate(-(int)clawOpenAngle, false);
		
		// lift blocks 5 cm and count block
		pulleyMotor.setSpeed(pulleySpeed);
		pulleyMotor.rotate((int)angleToLift, false);
		counter++;
	}
	
	/** Safely releases the tower on the ground.<br />
	 *  [WARNING] only call when robot is in the safe zone. */
	public void releaseTower(){
		
		// safety measure
		if(isInitialized == false){
			Sound.beep();
			System.out.println("FATAL ERROR: Claw has not been initialized.");
			return;
		}
		
		// angle to turn to place the tower on the ground
		double angleToRelease = computePulleyTurnAngle(safeLiftDistance);
		
		// lower tower to ground
		pulleyMotor.setSpeed(-pulleySpeed);
		pulleyMotor.rotate((int)angleToRelease, false);
		
		// release tower
		clawMotor.rotate((int)clawOpenAngle, false);
		Sound.beep();
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
	public int getCounter(){return this.counter;}
	
}
