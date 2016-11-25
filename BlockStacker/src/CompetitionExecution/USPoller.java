package CompetitionExecution;



import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

// Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  30 mS at the end of each loop, then one cycle through the loop
//  is approximately 50 mS.  This corresponds to a sampling rate
//  of 1/50mS or about 20 Hz.

/**
 * polls US sensor to check distance from objects
 * @author courtneywright
 *
 */
public class USPoller extends Thread{
	
	final static int FILTER_RESET_CONTROL = 3;   //number of values to be read to reset the filter 
	private SampleProvider us;
	private float[] usData;
	private double distance;

	
	public USPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
		us.fetchSample(usData,0);
	}

//  Sensors now return floats using a uniform protocol.

	
	public void run() {
		
		while (true) { 
			//edgeDetected = false;
			us.fetchSample(usData,0);					// acquire data
			distance=(usData[0]*100.0);					// extract from buffer
			if(distance > 255){
				distance = 255;
			}
			try { Thread.sleep(30); } catch(Exception e){}		// Poor man's timed sampling	
		}
	}
	
	/**
	 * Get US sensor reading.
	 * @return US sensor reading
	 */
	public double readUSDistance() {	
		return this.distance;
	}
	
	
	/**
	 * This method takes a cut off value that readings to be filtered on, and number of values to be discarded 
	 * to read the falling edge
	 * @param threshold
	 * @param filterOut, numbers of readings to be ignored when threshold is reached
	 * @return filtered distance Reading for a falling edge
	 */
	public double getFallingEdge(int threshold, int filterOut){
		int filterControlLow = 0, filterControlHigh = 0; 
		
		while(true){
			if (distance > threshold && filterControlHigh < FILTER_RESET_CONTROL) {
				// when robot reads values above the threshold, increase the filterControlUP only at first
				filterControlHigh++;
			}else if(distance > threshold) {
				// when robot keep getting the values above the threshold, then reset the filter
				filterControlHigh = 0;
				filterControlLow = 0;
			}
			
			if (distance <= threshold && filterControlLow < filterOut) {
				// when robot reads values below the threshold, increase the filterControlLOW only, which means to ignore them at first 
				filterControlLow++;
			} else if(distance <= threshold) {
				// when robot keep getting the cut off value below the threshold, then return the distance 
				return distance;
			}
			Delay.msDelay(50);     // keep checking frequency low to the pace of sampling frequency 20Hz
		}
	}
	
	
	/**
	 * This method takes a cut off value that readings to be filtered on, and number of values to be discarded 
	 * to read the rising edge
	 * @param threshold
	 * @param filterOut, numbers of readings to be ignored when threshold is reached
	 * @return filtered distance Reading for a rising edge
	 */
	public double getRisingEdge(int threshold, int filterOut){
		int filterControlLow = 0, filterControlHigh = 0; 
		
		while(true){
			if (distance >= threshold && filterControlHigh < filterOut) {
				// when robot reads values above the threshold, increase the filterControlHigh only at first
				filterControlHigh++;
			}else if(distance >= threshold) {
				// when robot keep getting the values above the threshold, then return the distance
				return distance;
			}
			
			if (distance < threshold && filterControlLow < FILTER_RESET_CONTROL) {
				// when robot reads values below the threshold, increase the filterControlLOW only, which means to ignore them at first 
				filterControlLow++;
			} else if(distance < threshold) {
				// when robot keep getting the cut off value below the threshold, then reset the filter
				filterControlHigh = 0;
				filterControlLow = 0;
			}
			Delay.msDelay(50);     // keep checking frequency low to the pace of sampling frequency 20Hz
		}
	}
	
	

}