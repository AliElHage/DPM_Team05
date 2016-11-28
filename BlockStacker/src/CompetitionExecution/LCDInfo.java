package CompetitionExecution;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * Gets values from each of the passed variables and writes them to the robot's display
 * to make it easier for human understanding of what the robot is doing.
 */
public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private USPoller frontUS, leftUS, rightUS;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	
	/**
	 * Array for displaying data
	 */
	private double [] pos;
	
	/**
	 * Constructor for LCDInfo to get values from each of the variables passed.
	 * @param odo odometer used to get values to write to screen
	 * @param frontUS front US sensor used to get values to write to screen
	 * @param leftUS left US sensor used to get values to write to screen
	 * @param rightUS right US sensor used to get values to write to screen
	 */
	public LCDInfo(Odometer odo, USPoller frontUS, USPoller leftUS, USPoller rightUS) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.frontUS = frontUS;
		this.leftUS = leftUS;
		this.rightUS = rightUS;
		
		/**
		 * Initializes the arrays for displaying data
		 */
		pos = new double [3];
		
		/** 
		 * Starts the timer used to decide when to refresh
		 */
		lcdTimer.start();
	}
	
	/**
	 * Starts LCD display
	 */
	public void initLCD(){
		lcdTimer.start();
	}
	
	/**
	 * Used to write odometer and US readings to screen by placing each variable at a proper
	 * location on the screen by refreshing to get new values.
	 */
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("frontUS: ", 0, 3);
		LCD.drawString("rightUS: ", 0, 4);
		LCD.drawString("leftUS: ", 0, 5);
		
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawInt((int)frontUS.readUSDistance(), 9, 3);
		LCD.drawInt((int)rightUS.readUSDistance(), 9, 4);
		LCD.drawInt((int)leftUS.readUSDistance(), 9, 5);
	}
}
