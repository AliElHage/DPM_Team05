package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ClawHandler {
	
	final static int clawSpeed = 100;					// speed of clawMotor
	final static int pulleySpeed = 300;					// speed of pulleyMotor
	final static double initialHeight =16.1;			// initial position of claws relative to the ground
	final static double minDistanceFromGround = 2.2;	// minimum distance of claw relative to ground
	final static double safeDropDistance = 9.8;			// amount to lower the claws to safely drop blocks on top of each other
	final static double motorRadius = 0.7;				// distance between the middle of the motor and the peripheral holes
	final static int clawOpenAngle = 50, clawCloseAngle = -10, clawSemiOpenAngle = 25;
	final static int clawAdjustingAngle = 10, graspOffsetDis = 2;
	final static double offsetLift = 0.3; 
	private Navigation nav;
	private double angleToRelease, angleToLift, angleToSet;	// amount by which to move the claws from their initial position for pulley	
	private EV3LargeRegulatedMotor pulleyMotor, clawMotor;	// all claw-related motors
	private int counter;									// counts the amount of blocks stacked
	private static int foamsCaptured;
	
	// NB FOR PULLEYSPEED: a negative speed is for lowering the claw; positive otherwise
	// NB FOR CLAWSOPENANGLE: a negative angle is for grabbing; releasing otherwise
	
	/** Constructor. */
	public ClawHandler(EV3LargeRegulatedMotor clawMotor, EV3LargeRegulatedMotor pulleyMotor, Navigation nav){
		this.clawMotor = clawMotor;
		this.pulleyMotor = pulleyMotor;
		this.counter=0;
		this.nav = nav;
		this.angleToRelease = this.computePulleyTurnAngle(initialHeight -minDistanceFromGround);
		this.angleToLift = this.computePulleyTurnAngle(initialHeight - minDistanceFromGround + offsetLift);
		this.angleToSet = this.computePulleyTurnAngle(safeDropDistance);
		this.pulleyMotor.setSpeed(pulleySpeed);
		this.clawMotor.setSpeed(clawSpeed);
		this.pulleyMotor.rotate(-3);
		this.open();
		foamsCaptured = 0;
	}

	
	/**
	 * open the claw when it is fully closed
	 */
	public void open(){
		clawMotor.rotateTo((int)clawOpenAngle, false);
	}
	
	/**
	 * close the claw
	 */
	public void close(){
		clawMotor.rotateTo(clawCloseAngle, false);	
	}
	
	/**
	 * semi-open the claw for adjusting foam block position  
	 */
	public void semiOpen(){
		clawMotor.rotateTo(clawSemiOpenAngle, false);	
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
			
			//release the claw down
			this.putDown();
			
			// adjust the foam block position and grasp the block
			this.fixAndGrasp();
			
			// lifts the block up
			this.pullUp();
			
			// count block
			counter++;
			foamsCaptured++;
			return;
		}
		
		// gently place the held block on top of the other block
		this.putDownToObj();
		
		// release block from claws and lower the claw to ground block level
		this.open();				///**************************
		this.putDownToBot();
		
		// grab ground block
		this.fixAndGrasp();
		
		// lift blocks up
		this.pullUp();
		counter++;
		foamsCaptured++;
	}
	
	/** Safely releases the tower on the ground.<br />
	 *  [WARNING] only call when robot is in the safe zone. */
	public void releaseTower(){
		
		// lower tower to ground
		this.putDown();
		
		// release tower
		this.open();			//********************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
	
	/**
	 * fix position of foam block and grasp it 
	 */
	public void fixAndGrasp(){
		this.close();
		this.semiOpen();
		for(int i=0;i<3;i++){
			nav.turn(-clawAdjustingAngle);
			nav.turn(clawAdjustingAngle);
		}
		nav.goForward(graspOffsetDis);
		this.close();
	} 
	
	
	/** Gets the amount of blocks held by the robot. */
	public static int getfoamsCaptured(){
		return foamsCaptured;
	}
	
}
