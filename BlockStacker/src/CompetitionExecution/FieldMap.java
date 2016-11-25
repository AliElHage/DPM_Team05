package CompetitionExecution;

import java.util.ArrayList;
import java.util.List;

import CompetitionExecution.Grid.Status;


/**
 * This class serves as a helper for navigation; it will depict the competition field as 10*10 grids,
 * and it will mark the prohibited area that robot should take a detour around, such as detected wooden
 * blocks and a designated zone
 * @author joey
 *
 */
public class FieldMap {
	
	final static double GRID_LENGTH=30.1;

	public static List<Grid> fieldMap = new ArrayList<>(); 
	
	public FieldMap(){
		for(int i=0; i<10; i++){
			for(int j=0; j<10; j++){
				Grid grid = new Grid(i,j);
				grid.setStatus(Status.CLEAR);
				fieldMap.add(grid);		// put 10*10 grids into the collection of grids
				
			}
		}
	}
	
	/**
	 * This method will mark the grid which contains the point in coordinate system of odometer as blocked
	 * @param pointX
	 * @param pointY
	 */
	public void markBlocked(double pointX, double pointY){
		this.getGrid(pointX, pointY).setBlocked();
	}
	
	/**
	 * This method will check if the grid which contains the point in coordinate system of odometer is blocked or not.
	 * @param pointX
	 * @param pointY
	 * @return boolean
	 */
	public boolean checkBlocked(double pointX, double pointY){
		return this.getGrid(pointX, pointY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * This method will check if the grid is blocked or not.
	 * @param gridX
	 * @param gridY
	 * @return boolean
	 */
	public boolean checkBlocked(int gridX, int gridY){
		return this.getGrid(gridX, gridY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * This method will check if the grid is blocked or not.
	 * @param grid
	 * @return boolean
	 */
	public boolean checkBlocked(Grid grid){
		return grid.getStatus().equals(Grid.Status.BLOCKED);
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
	 * This method return a collection of grids as zone by passing the lower left and upper right gridINDEX
	 * only for the designated zone only 
	 * @param LZx lower left x
	 * @param LZy lower left y
	 * @param UZx upper right x
	 * @param UZy upper right y
	 * @return a collection of grids which depict a zone 
	 */
	public ArrayList<Grid> getZone(int LZx, int LZy, int UZx, int UZy){
		/*UZx--;
		UZy--;  //fixed passed parameter to fix the map GridIndex*/
		ArrayList<Grid> zone = new ArrayList<>();
		for(int i=LZx;i<UZx;i++){
			for(int y=LZy;y<UZy;y++){
				zone.add(this.getGrid(i, y));
			}
		}
		return zone;
	}
	
	/**
	 * The method marked all the grids in the designated zone as blocked by passing the lower left and upper right gridINDEX 
	 * @param LZx lower left x
	 * @param LZy lower left y
	 * @param UZx upper right x
	 * @param UZy upper right y
	 */
	public void zoneBlocked(int LZx, int LZy, int UZx, int UZy){
		ArrayList<Grid> zone = this.getZone(LZx, LZy, UZx, UZy);
		for(Grid grid: zone){
			grid.setBlocked();
		}
	}
	
	/**
	 * This method return a collection of grids which depict the outer border of the a zone
	 * @param LZx lower left x
	 * @param LZy lower left y
	 * @param UZx upper right x
	 * @param UZy upper right y
	 * @return a collection of grids  which depict the border
	 */
	public ArrayList<Grid> getBorder(int LZx, int LZy, int UZx, int UZy){
		ArrayList<Grid> largerZone = this.getZone(LZx-1, LZy-1, UZx+1, UZy+1);
		ArrayList<Grid> zone = this.getZone(LZx, LZy, UZx, UZy);
		for(Grid grid:zone){
			largerZone.remove(grid);
		}
		return this.fixBorder(largerZone);
	}
	
	/**
	 * helper method for getBorder to put grids comprise the border in a continous order
	 * @param border
	 * @return organized border
	 */
	private  ArrayList<Grid> fixBorder(ArrayList<Grid> border){
		ArrayList<Grid> newBorder = new ArrayList<>();
		newBorder.add(border.get(0));
		int index=0;
		int size = border.size();
		while(newBorder.size()!=size){
			for(Grid grid: border){
				if(this.isNexTo(newBorder.get(index),grid)){
					newBorder.add(grid);
					border.remove(grid);
					index++;
					break;
				}
			}
		}
		return newBorder;
	}
	
	
	/**
	 * This method checks if grids in a collection is continuous to depict a continuous path
	 * @param grids
	 * @return boolean
	 */
	public boolean isContinuous(ArrayList<Grid> grids){
		int index = 0;
		//iterate through the collection and return false if either x or y doesn't vary by 1 compared to the adjacent
		for(Grid grid:grids){
			if(!this.isNexTo(grid, grids.get(++index%grids.size()))){
				return false;
			}
		}
		return true;
	}
	
	/**
	 * This method checks if two grids are next to each other
	 * @param grid1
	 * @param grid2
	 * @return boolean 
	 */
	private boolean isNexTo(Grid grid1, Grid grid2){
		if(Math.abs(grid1.getGridX() - grid2.getGridX())!=1 &&
				Math.abs(grid1.getGridY() - grid2.getGridY())!=1){
			return false;		//return false if both grid y and x vary not 1
		}else if(Math.abs(grid1.getGridX() - grid2.getGridX())==1 &&
				Math.abs(grid1.getGridY() - grid2.getGridY())==1){
			return false;		//return false if both grid y and x vary 1
		}else if(grid1.equals(grid2)){
			return false;		//return false if grids are the same object
		}
		return true;
	}
	
		
	/**
	 * This method removes all the repeated element grids in a path.
	 * @param path 
	 */
	public void revisePath(ArrayList<Grid> path){
		for(int i=0;i<path.size()-1;i++){
			for(int j=i+1;j<path.size();j++){
				if(path.get(i).equals(path.get(j))){
					path.remove(j);
				}
			}
		}
	}
	
	
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
	
	
}


class Grid {
	
	enum Status {CLEAR,BLOCKED}		//define three status of grid: clear, blocked or null for unchecked
	
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
	
	public void setBlocked(){
		this.status = Status.BLOCKED;
	}

	public int[] getGridIndex() {
		return gridIndex;
	}
	
	public int getGridX(){
		return gridIndex[0];
	}
	
	public int getGridY(){
		return gridIndex[1];
	}
	
	
	/**
	 * to compare this grid with reference grid by passing index of reference grid
	 * @param targetX
	 * @param targetY
	 * @return true if equal
	 */
	public boolean equals(int targetX, int targetY){
		if(gridIndex[0] == targetX &&  gridIndex[1] == targetY){
			return true;
		}else{
			return false;
		}
	}
}