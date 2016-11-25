package CompetitionExecution;

/**
 * Timer class keeps track of time for competition 
 * @author joey
 *
 */
public final class Timer {
	private static long timeLimit;	//store the time limit in second
	private static long timeStart;	//store the starting time in milisecond
	
	
	private Timer(){}	//private constructor ensures time consistency
	
	/**
	 * This method takes the time limit in second and starts the timing 
	 * @param limit 
	 */
	public static void startTiming(long limit){
		timeStart = System.currentTimeMillis();
		timeLimit = limit;
	}
	
	/**
	 * This method return the time left in seconds
	 * @return time left in seconds
	 */
	public static long timeLeft(){
		long timeElapsed = System.currentTimeMillis() - timeStart;
		return timeLimit - timeElapsed/1000;
	}
}
