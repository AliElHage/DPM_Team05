package CompetitionExecution;

import java.io.IOException;
import java.util.HashMap;

import CompetitionExecution.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;

/**
 * interprets everything received from wifi and determines what to do
 * @author courtneywright
 *
 */
public class ParameterInterpretation {
	
	private static final String SERVER_IP = "192.168.2.16";
	private static final int TEAM_NUMBER = 5;
	private static TextLCD LCD = LocalEV3.get().getTextLCD();

	private int BTN, BSC, CTN, CSC, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	
	public ParameterInterpretation() {
		
	}
	
	public void interpret() {
		WifiConnection conn = null;
		
		try {
			System.out.println("Connecting...");
			conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, true);
		} catch (IOException e) {
			System.out.println("Connection failed");
		}
		
		LCD.clear();
		
		if (conn != null) {
			HashMap<String, Integer> t = conn.StartData;
			if (t == null) {
				System.out.println("Failed to read transmission");
			} else {
//				System.out.println("Transmission read:\n" + t.toString());
				Main.BTN = t.get("BTN");
				Main.BSC = t.get("BSC");
				Main.CTN = t.get("CTN");
				Main.CSC = t.get("CSC");
				Main.LRZx = t.get("LRZx");
				Main.LRZy = t.get("LRZy");
				Main.URZx = t.get("URZx");
				Main.URZy = t.get("URZy");
				Main.LGZx = t.get("LGZx");
				Main.LGZy = t.get("LGZy");
				Main.UGZx = t.get("UGZx");
				Main.UGZy = t.get("UGZy");
			}
		}

	}
}