package searching;



import lejos.robotics.SampleProvider;

//
// Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  50 mS at the end of each loop, then one cycle through the loop
//  is approximately 70 mS.  This corresponds to a sampling rate
//  of 1/70mS or about 14 Hz.
//


public class UltrasonicPoller extends Thread{
	private SampleProvider us;
	private float[] usData;
	private double distance;

	
	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
		us.fetchSample(usData,0);
	}

//  Sensors now return floats using a uniform protocol.
//  Need to convert US result to an integer [0,255]
	
	public void run() {
		
		while (true) { 
			//edgeDetected = false;
			us.fetchSample(usData,0);							// acquire data
			distance=(usData[0]*100.0);					// extract from buffer
			if(distance > 255){
				distance = 255;
			}
 

			//try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling	//original 50
		}
	}
	
	public double readUSDistance() {	
		return this.distance;
	}
	
	

}