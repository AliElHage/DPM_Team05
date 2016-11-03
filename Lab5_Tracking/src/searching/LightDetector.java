package searching;

import lejos.robotics.SampleProvider;

public class LightDetector extends Thread{
	final static int SIGN_BLUE = 12, SING_WOOD =6; //Val[2] for RGB
	
	private SampleProvider colorSensor;
	private float[] colorData;	
	private int colorReading;
	
	public LightDetector(SampleProvider colorSensor, float[] colorData) {
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
	
}

