package CompetitionExecution;


/**
 * This thread should run after robot has successfully localized, and it should keep running throughout the competition to 
 *  keep checking robot current position in order not to exceed the 10*10 grid field. 
 *   
 * @author joey
 *
 */
public class BorderMonitor extends Thread{
	
	private Navigation nav;
	private static boolean isDone;
	
	public BorderMonitor(){
		isDone = false;
	}
	
	
	public void run(){
		while(!isDone){
			if(nav.odometer.getX() < 0 || nav.odometer.getX() > 300 || 
					nav.odometer.getY() < 0 || nav.odometer.getY() > 300){
				nav.interruptTraveling();
			}
		}
	}
	
	public static void stopChecking(){
		isDone = true;
	}

}
