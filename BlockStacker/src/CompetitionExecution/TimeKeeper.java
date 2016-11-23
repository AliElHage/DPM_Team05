package CompetitionExecution;

/**
 * 
 * @author joey
 *
 */
public class TimeKeeper extends Thread{
	
	private Navigation nav;
	private StartCorner startCorner;
	
	public TimeKeeper(StartCorner startCorner){
		this.startCorner = startCorner;
	}
	
	public void run(){
		while(true){
			
			if(Timer.timeLeft() < 60){
				nav.interruptTraveling();
				BorderMonitor.stopChecking();
				nav.travelTo(startCorner.getX(), startCorner.getY());	//fix later, should travel to the center of grid ***********
			}
		}
	}
}
