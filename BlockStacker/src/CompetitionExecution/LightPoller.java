package CompetitionExecution;

import lejos.robotics.SampleProvider;

/**
 * checks color of block to see if it is the right one
 * @author courtneywright
 *
 */
public class LightPoller extends Thread{
	final static int SIGN_BLUE = 12, SING_WOOD =6; //Val[2] for RGB
	
	private SampleProvider colorSensor;
	private float[] colorData;	
	private int colorReading;
	
	public LightPoller(SampleProvider colorSensor, float[] colorData) {
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	public void run(){
		while(true){
			colorSensor.fetchSample(colorData,0);
			colorReading = (int)(colorData[2] *100.0); 
		}
	}
	
	public boolean colorCheck() {
		if (colorReading > SIGN_BLUE){			
			return true;
		}else {
			return false;
		}
	}
	
	public void colorTesting(){
		if(this.colorCheck()){
			System.out.println("Blue Styrofoam  block");
		}else{
			System.out.println("Block");
		}
	}
	
	public int readColor(){
		return colorReading;
	}
	
	/**
	 * polls color sensor to get the data to see if it passes over a line.
	 * returns a boolean true if it does cross a line.
	 */
	public boolean lineCrossed(){
		colorSensor.fetchSample(colorData, 0);
		
		if(colorData[0] < 0.30){
			return true;
		}
		else{
			return false;
		}
	}
	
}

