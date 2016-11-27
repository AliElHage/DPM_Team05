package CompetitionExecution;

/**
 * This class should run throughout the competition to keep checking timeLeft and will drive robot home at the end of game
 * 
 * @author joey
 *
 */
public class TimeKeeper extends Thread{
	
	private Navigation nav;
	private BlockHunter hunter;
	
	public TimeKeeper(Navigation nav, BlockHunter blockHunter){
		this.nav = nav;
		this.hunter = blockHunter;
	}
	
	public void run(){
		while(true){
			
			if(Timer.timeLeft() < 60){
				nav.interruptTraveling();
				hunter.stopHunting();
				BorderMonitor.stopChecking();
				nav.goHome();	//fix later, should travel to the center of grid ***********
				return;
			}
		}
	}
}
