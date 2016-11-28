package CompetitionExecution;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Handles all claw movements including opening, closing, hoisting, and lowering
 */
public class ClawHandler {
	
	/**
	 *  speed of clawMotor
	 */
	final static int clawSpeed = 100;
	/**
	 *  speed of pulleyMotor
	 */
	final static int pulleySpeed = 300;	
	/**
	 *  initial position of claws relative to the ground
	 */
	final static double initialHeight =16.1;	
	/**
	 *  minimum distance of claw relative to ground
	 */
	final static double minDistanceFromGround = 2.2;
	/**
	 *  amount to lower the claws to safely drop blocks on top of each other
	 */
	final static double safeDropDistance = 9.8;		
	/**
	 *  distance between the middle of the motor and the peripheral holes
	 */
	final static double motorRadius = 0.7;	
	
	final static int clawOpenAngle = 50, clawCloseAngle = -10, clawSemiOpenAngle = 25;
	final static int clawAdjustingAngle = 10, graspOffsetDis = 2;
	final static double offsetLift = 0.3; 
	private Navigation nav;
	
	/**
	 *  amount by which to move the claws from their initial position for pulley	
	 */
	private double angleToRelease, angleToLift, angleToSet;	
	/**
	 *  all claw-related motors
	 */
	private EV3LargeRegulatedMotor pulleyMotor, clawMotor;	
	/**
	 *  counts the amount of blocks stacked
	 */
	private int counter;	
	/**
	 *  counts the amount of foams captured
	 */
	private static int foamsCaptured;
	
	/** NB FOR PULLEYSPEED: a negative speed is for lowering the claw; positive otherwise */
	/** NB FOR CLAWSOPENANGLE: a negative angle is for grabbing; releasing otherwise */
	
	/**
	 * constructor for Claw Handler that can open, lower, and raise the claw to catch a styrofoam block
	 * @param clawMotor motor used to open and close claw
	 * @param pulleyMotor motor used to hoist claw up and let it down
	 * @param nav navigator used to adjust robot when grabbing a block
	 */
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
	 * Open the claw when it is fully closed.
	 */
	public void open(){
		clawMotor.rotateTo((int)clawOpenAngle, false);
	}
	
	/**
	 * Close the claw fully.
	 */
	public void close(){
		clawMotor.rotateTo(clawCloseAngle, false);	
	}
	
	/**
	 * Semi-open the claw for adjusting foam block position.  
	 */
	public void semiOpen(){
		clawMotor.rotateTo(clawSemiOpenAngle, false);	
	}
	
	/**
	 * Release the pulley down all the way down to the bottom.  
	 */
	public void putDown(){
		pulleyMotor.rotate((int)angleToRelease, false);
	}
	
	/**
	 * Release the pulley down to the top of the foam block.
	 */
	public void putDownToObj(){
		pulleyMotor.rotate((int)angleToSet, false);
	}
	
	
	/**
	 * Release the pulley down to bottom from the height of the foam block. 
	 */
	public void putDownToBot(){
		double angleToGrab = computePulleyTurnAngle(initialHeight - minDistanceFromGround - safeDropDistance);
		pulleyMotor.rotate((int)angleToGrab, false);
	}
	
	/**
	 * Pull pulley all the way up the top. 
	 */
	public void pullUp(){
		pulleyMotor.rotate(-(int)angleToLift, false);
	}
	
	/** 
	 *  Release the claw and grasp the foam and pull up the claw. 
	 *	Only call when the robot is at the appropriate distance.
	 *  (block should be below the claws before this method is called).
	 */
	public void grasp(){
		
		/**
		 * Accounts for case when there is no block in the claw, so it does
		 * not need to be treated as gently as when there is already at least
		 * one block in the claw. Puts claw down, adjusts robot, grabs block,
		 * pulls claw up, and updates block count to 1.
		 */
		if(counter == 0){
			
			this.putDown();
			this.fixAndGrasp();
			this.pullUp();
			
			counter++;
			foamsCaptured++;
			return;
		}
		
		/**
		 * Accounts for case when there is at least one block in the claw. Places
		 * block down and releases before trying to grab and hoist stack of blocks.
		 */		
		this.putDownToObj();
		this.open();				///**************************
		
		this.putDownToBot();
		this.fixAndGrasp();
		this.pullUp();
		
		counter++;
		foamsCaptured++;
	}
	
	/**
	 * Handles case where robot needs to drop final stack of styrofoam blocks in proper zone. 
	 * Safely releases the tower on the ground, releases, and updates block count to 0. <br />
	 * [WARNING] only call when robot is in the safe zone.
	 */
	public void releaseTower(){

		this.putDown();
		
		this.open();			//********************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		this.counter = 0;
		Sound.beep();
		this.pullUp();
	}
	
	
	/** 
	 * Computes angle by which the pulleyMotor should rotate to lift the claws by distanceToRotate. 
	 */
	public double computePulleyTurnAngle(double distanceToRotate){
		
		/** 
		 * Computes angle by which the pulleyMotor should rotate to lift the claws by distanceToRotate. 
		 */
		double motorCircumference = 2 * Math.PI * motorRadius;
		
		/** 
		 * Compute the circumference described by a full turn of the motor.
		 */
		double angle = distanceToRotate / motorCircumference * 360;
		
		return angle;
	}
	
	/**
	 * Prevention for improper block positioning when robot tries to grasp styrofoam.
	 * Attempts to grasp block, opens, adjusts robot to shift block, moves forward,
	 * and grabs block.
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
	
	/** 
	 * Gets the amount of blocks held by the robot. 
	 */	public static int getfoamsCaptured(){
		return foamsCaptured;
	}
	
}
