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

	public static List<Grid> grids = new ArrayList<>(); 
	
	public FieldMap(){
		for(int i=0; i<10; i++){
			for(int j=0; j<10; j++){
				grids.add(new Grid(i,j));		// put 10*10 grids into the collection of grids
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
		return this.getGrid(pointX, pointY).equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * This method will check if the grid is blocked or not.
	 * @param gridX
	 * @param gridY
	 * @return
	 */
	public boolean checkBlocked(int gridX, int gridY){
		return this.getGrid(gridX, gridY).equals(Grid.Status.BLOCKED);
	}
	
	
	
	private Grid getGrid(double pointX, double pointY){
		int[] targetGrid = convertPointToGrid(pointX, pointY);
		
		for(Grid grid: grids){			//iterate through the grids collection and find the target grid
			if(grid.getGridIndex().equals(targetGrid)){
				return grid;
			}
		}
		return null;
	}
	
	private Grid getGrid(int gridX, int gridY){
		int[] targetGrid = {gridX, gridY};
		
		for(Grid grid: grids){			//iterate through the grids collection and find the target grid
			if(grid.getGridIndex().equals(targetGrid)){
				return grid;
			}
		}
		return null;
	}
	
	
	/**
	 * This method will convert the coordinates in odometer to the grid index in the fieldMap.
	 * @param pointX
	 * @param pointY
	 * @return
	 */
	private int[] convertPointToGrid(double pointX, double pointY){
		int gridX = (int)(pointX%GRID_LENGTH);
		int gridY = (int)(pointY%GRID_LENGTH);
		return new int[] {gridX, gridY};
	}
	
}

class Grid {
	
	enum Status {CLEAR,BLOCKED}		//define three states of robot when it is hunting
	
	private int[] gridIndex;		//[0] for x, [1] for y axis of grids
	private Status status;
	
	public Grid(){}					//empty constructor
	
	public Grid(int gridX, int gridY)  {  
		gridIndex = new int[2];		
		gridIndex[0] = gridX;
		gridIndex[1] = gridY;
	}

	public Status getStatus() {
		return status;
	}

	public void setStatus(Status status) {
		this.status = status;
	}

	public int[] getGridIndex() {
		return gridIndex;
	}
	
	
	
	
}