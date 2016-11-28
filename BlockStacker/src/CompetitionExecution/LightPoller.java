package CompetitionExecution;

import lejos.robotics.SampleProvider;

/**
 * Implemented in localization to check the red color value in order to determine 
 * if a line has been crossed.
 */
public class LightPoller extends Thread{
	final static int SIGN_BLUE = 12, SING_WOOD =6; //Val[2] for RGB
	
	private SampleProvider colorSensor;
	private float[] colorData;	
	private int colorReading;
	
	/**
	 * Constructor for Light Poller to pass sample provider and data storage float array
	 * @param colorSensor light sensor
	 * @param colorData array used to store data received by the light sensor
	 */
	public LightPoller(SampleProvider colorSensor, float[] colorData) {
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	/**
	 * Used to continuously poll the color sensor and store data in colorData array.
	 */
	public void run(){
		while(true){
			colorSensor.fetchSample(colorData,0);
			colorReading = (int)(colorData[0] *100.0); 
		}
	}
	
	/**
	 * Checks if a block is blue or not by comparing the color data from the sensor
	 * with the know value of blue for a block.
	 * @return if block is identified to be blue in color, return T, if not, return F
	 */
	public boolean colorCheck() {
		if (colorReading > SIGN_BLUE){			
			return true;
		}else {
			return false;
		}
	}
	
	/**
	 * Makes it clear to tester using robot which block has been detected by printing the 
	 * type of block in the out file when a block is detected, for ease of testing.
	 */
	public void colorTesting(){
		if(this.colorCheck()){
			System.out.println("Blue Styrofoam  block");
		}else{
			System.out.println("Block");
		}
	}
	
	/**
	 * Getter method for color reading
	 * @return color reading
	 */
	public int readColor(){
		return colorReading;
	}
	
	/**
	 * Determines if a line has been crossed by comparing the color reading from the sensor
	 * to know value of black line. 
	 * @return if a line was detected, return T, if not, return F
	 */
	public boolean lineCrossed(){
		if(colorReading < 30){
			return true;
		}
		else{
			return false;
		}
	}
	
}

