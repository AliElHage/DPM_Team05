package CompetitionExecution;


/**
 * This thread should run after robot has successfully localized, and it should keep running throughout the competition to 
 *  keep checking robot's current position in order not to exceed the 10*10 grid field. 
 */
public class BorderMonitor extends Thread{
	
	private Navigation nav;
	private BlockHunter hunter;
	private static boolean isDone;
	
	/**
	 * Constructor used to ensure robot does not travel off of grid.
	 * @param nav navigator used to stop robot from going out of range
	 * @param blockHunter block hunter used to check if robot has captured styrofoam blocks
	 */
	public BorderMonitor(Navigation nav, BlockHunter blockHunter){
		this.nav = nav;
		this.hunter =blockHunter;
		isDone = false;
	}
	
	/**
	 * Used to ensure robot stays in grid by checking its location on the map and interrupting
	 * movement if necessary to stop it from trying to move out of competition environment.
	 */
	public void run(){
		while(!isDone){
			/**
			 * If robot has crossed the map border interrupt navigation, and stop hunting for
			 * blocks. 
			 */
			if(nav.odometer.getX() < 0 || nav.odometer.getX() > 300 || 			
					nav.odometer.getY() < 0 || nav.odometer.getY() > 300){
				nav.interruptTraveling();
				hunter.stopHunting();
				
				/**
				 * If robot has captured foam blocks, call stopChecking.
				 */
				if(hunter.foamsCaptured()){		 
					stopChecking();			
					
				}
				
			}
		}
	}
	
	/**
	 * Stops checking if the competition is over by changing isDone boolean.
	 */
	public static void stopChecking(){
		isDone = true;
	}

}
