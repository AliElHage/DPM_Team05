package CompetitionExecution;

import java.util.ArrayList;
import java.util.List;


/**
 * This class serves as a helper for navigation; it will depict the competition field as 10*10 grids,
 * and it will mark the prohibited area that robot should take a detour around, such as detected wooden
 * blocks and a designated zone
 * @author joey
 *
 */
public class FieldMap {
	
	final static double GRID_LENGTH=30.48;

	public static List<Grid> fieldMap = new ArrayList<>(); 
	
	public FieldMap(){
		for(int i=0; i<10; i++){
			for(int j=0; j<10; j++){
				fieldMap.add(new Grid(i,j));		// put 10*10 grids into the collection of grids
			}
		}
	}
	
	/**
	 * This method will mark the grid which contains the point in coordinate system of odometer as blocked
	 * @param pointX
	 * @param pointY
	 */
	public void markBlocked(double pointX, double pointY){
		this.getGrid(pointX, pointY).setStatus(Grid.Status.BLOCKED);
	}
	
	
	/**
	 * This method will check if the grid which contains the point in coordinate system of odometer is blocked or not.
	 * @param pointX
	 * @param pointY
	 * @return
	 */
	public boolean checkBlocked(double pointX, double pointY){
		return this.getGrid(pointX, pointY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * This method will check if the grid is blocked or not.
	 * @param gridX
	 * @param gridY
	 * @return
	 */
	public boolean checkBlocked(int gridX, int gridY){
		return this.getGrid(gridX, gridY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	
	/**
	 * This method to get the grid object by the coordinate of odometer readings
	 * @param pointX
	 * @param pointY
	 * @return Grid
	 */
	public Grid getGrid(double pointX, double pointY){
		int[] targetGrid = convertPointToGrid(pointX, pointY);
		return this.getGrid(targetGrid[0], targetGrid[1]);
	}
	
	/**
	 * This method to get the grid object by the grid index
	 * @param gridX
	 * @param gridY
	 * @return Grid
	 */
	public Grid getGrid(int gridX, int gridY){
		
		for(Grid grid: fieldMap){			//iterate through the grids collection and find the target grid
			if(grid.equals(gridX, gridY)){
				return grid;
			}
		}
		return null;
	}
	
	/**
	 * This method return a collection of grids as zone by passing the lower left and upper right points 
	 * @param LZx lower left x
	 * @param LZy lower left y
	 * @param UZx upper right x
	 * @param UZy upper right y
	 * @return
	 */
	/*public ArrayList<Grid> getZone(double LZx, double LZy, double UZx, double UZy){
		
	}*/
	
	
	/**
	 * This method will convert the coordinates in odometer reading to the grid index in the fieldMap.
	 * @param pointX
	 * @param pointY
	 * @return int[] 
	 */
	public static int[] convertPointToGrid(double pointX, double pointY){
		int gridX = (int)(pointX/GRID_LENGTH);
		int gridY = (int)(pointY/GRID_LENGTH);
		return new int[] {gridX, gridY};
	}
	
	
	/**
	 * This method will convert grid index in the fieldMap to the the coordinates of the point centered at that grid.
	 * @param gridX
	 * @param gridY
	 * @return double[] 
	 */
	public static double[] convertGridToPoint(int gridX, int gridY){
		double pointX = GRID_LENGTH/2 + gridX*GRID_LENGTH; 
		double pointY = GRID_LENGTH/2 + gridY*GRID_LENGTH; 
		return new double[] {pointX,pointY};	
	}
	
	public List<Grid> getFieldMap(){
		return this.fieldMap;
	}
	
	
	/*public static */
	
}

	
	
	
	
