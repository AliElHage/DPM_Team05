package CompetitionExecution;

import java.util.ArrayList;
import java.util.List;

import CompetitionExecution.Grid.Status;


/**
 * Serves as a helper for navigation by depicting the competition field as a 10*10 grid,
 * and marking areas that robot should take a detour around, such as a 
 * square with a detected wooden block and a the other team's designated zone.
 */
public class FieldMap {
	
	/**
	 * measured length of a grid square
	 */
	final static double GRID_LENGTH=30.1;

	/**
	 * holder for fieldMap
	 */
	public static List<Grid> fieldMap = new ArrayList<>(); 
	
	/**
	 * FieldConstructor. Creates and instance of the class that sets up a data type
	 * to hold a physical representation of the playing field.
	 */
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
	 * Marks the grid which contains the point in coordinate system of odometer as blocked.
	 * @param pointX x-coordinate of grid square in which the robot should not go
	 * @param pointY y-coordinate of grid square in which the robot should not go
	 */
	public void markBlocked(double pointX, double pointY){
		this.getGrid(pointX, pointY).setBlocked();
	}
	
	/**
	 * Makes it possible to check if a specific point in the coordinate system of the odometer
	 * is blocked by calling helper methods to convert the point to convert the point to
	 * a grid square and checking the status of this square.
	 * @param pointX x-coordinate of point in grid square that is being checked
	 * @param pointY y-coordinate of point in grid square that is being checked
	 * @return boolean if grid square is blocked T, if not F
	 */
	public boolean checkBlocked(double pointX, double pointY){
		return this.getGrid(pointX, pointY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * Calls helper methods to convert x,y-coordinates of a grid square into the square itself
	 * and checks the status of that square to see if it is blocked or not.
	 * @param gridX x-coordinate of grid square that is being checked
	 * @param gridY y-coordinate of grid square that is being checked
	 * @return boolean if grid square is blocked T, if not F
	 */
	public boolean checkBlocked(int gridX, int gridY){
		return this.getGrid(gridX, gridY).getStatus().equals(Grid.Status.BLOCKED);
	}
	
	/**
	 * Checker method to see if a grid square is blocked or not.
	 * @param grid grid square that is being checked
	 * @return boolean if a grid square is blocked T, if not F
	 */
	boolean checkBlocked(Grid grid){
		return grid.getStatus().equals(Grid.Status.BLOCKED);
	}
	

	/**
	 * Getter method for a grid square using the x,y-coordinates in the odometer
	 * coordinate system.
	 * @param pointX x-coordinate of grid square
	 * @param pointY y-coordinate of grid square
	 * @return Grid grid square
	 */
	public Grid getGrid(double pointX, double pointY){
		int[] targetGrid = convertPointToGrid(pointX, pointY);
		return this.getGrid(targetGrid[0], targetGrid[1]);
	}
	
	/**
	 * Getter method for a grid square using the x,y-coordinates of said grid square
	 * @param gridX x-coordinate of grid square
	 * @param gridY y-coordinate of grid square
	 * @return Grid grid square
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
	 * Returns a collection of grid squares representing a zone by passing the lower left 
	 * and upper right gridINDEX to easily group square in a zone for the designated zone. 
	 * @param LZx lower left x x-coordinate of lower left corner of zone
	 * @param LZy lower left y y-coordinate of lower left corner of zone
	 * @param UZx upper right x x-coordinate of upper right corner of zone
	 * @param UZy upper right y y-coordinate of upper right corner of zone
	 * @return zone a collection of grid squares which depict a zone 
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
	 * Marks all the grid squares in the designated zone as blocked 
	 * by passing the lower left and upper right gridINDEX to make it simple
	 * to block off the entire zone of the other team. 
	 * @param LZx lower left x x-coordinate for lower left corner of zone
	 * @param LZy lower left y y-coordinate for lower left corner of zone
	 * @param UZx upper right x x-coordinate for upper right corner of zone
	 * @param UZy upper right y y-coordinate for upper right corner of zone
	 */
	public void zoneBlocked(int LZx, int LZy, int UZx, int UZy){
		ArrayList<Grid> zone = this.getZone(LZx, LZy, UZx, UZy);
		for(Grid grid: zone){
			grid.setBlocked();
		}
	}
	
	/**
	 * Easily finds the border of a zone by returning a collection of grid squares 
	 * which depict the outer border of the zone.
	 * @param LZx lower left x x-coordinate for lower left corner of zone
	 * @param LZy lower left y y-coordinate for lower left corner of zone
	 * @param UZx upper right x x-coordinate for upper right corner of zone
	 * @param UZy upper right y y-coordinate for upper right corner of zone
	 * @return a collection of grids  which depict the border
	 */
	public ArrayList<Grid> getZoneBorder(int LZx, int LZy, int UZx, int UZy){
		ArrayList<Grid> largerZone = this.getZone(LZx-1, LZy-1, UZx+1, UZy+1);
		ArrayList<Grid> zone = this.getZone(LZx, LZy, UZx, UZy);
		for(Grid grid:zone){
			largerZone.remove(grid);
		}
		return this.fixBorder(largerZone);
	}
	
	/**
	 * Helper method for getBorder to put grids comprise the border in a continuous order
	 * @param border list of grid squares found to make up the border of the zone
	 * @return newBorder organized list of same grid squares that make up border of zone
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
	 * Ensures a chosen path is continuous by checking if a collection of grid squares is continuous.
	 * @param grids list of grid squares
	 * @return boolean if the squares are continuous T, if not F
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
	 * Checks if two grid squares are next to each other
	 * @param grid1 first grid square to check
	 * @param grid2 second grid square to compare to
	 * @return boolean if the squares are the adjacent T, if not F
	 */
	private boolean isNexTo(Grid grid1, Grid grid2){
		/**
		 * return false if both grid y and x vary not 1
		 */
		if (Math.abs(grid1.getGridX() - grid2.getGridX())!=1 &&
				Math.abs(grid1.getGridY() - grid2.getGridY())!=1){
			return false;		
		}
		/**
		 * return false if both grid y and x vary 1
		 */
		else if (Math.abs(grid1.getGridX() - grid2.getGridX())==1 &&
				Math.abs(grid1.getGridY() - grid2.getGridY())==1){
			return false;		
		}
		/**
		 * return false if grids are the same object
		 */
		else if (grid1.equals(grid2)){
			return false;		//return false if grids are the same object
		}
		return true;
	}
	
		
	/**
	 * Allows robot to move correctly and efficiently by removing all 
	 * repeated element grid squares in a path.
	 * @param path currently chosen path to travel
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
	 * Simplifies method calls by converting a point in the odometer's coordinate system, given
	 * its x,y-coordinates into a grid square.
	 * @param pointX x-coordinate of point
	 * @param pointY y-coordinate of point
	 * @return int[] array containing x-coordinate at [0] and y-coordinate at [1] of square
	 */
	public static int[] convertPointToGrid(double pointX, double pointY){
		int gridX = (int)(pointX/GRID_LENGTH);
		int gridY = (int)(pointY/GRID_LENGTH);
		return new int[] {gridX, gridY};
	}	
	
	/**
	 * Simplifies method calls by converting a grid square, given its x,y-coordinates
	 * into the point centered in that grid square, in the odometer's coordinate system.
	 * @param gridX x-coordinate of grid square
	 * @param gridY y-coordinate of grid square
	 * @return double[] array containing x-coordinate at [0] and y-coordinate at [1] of point
	 */
	public static double[] convertGridToPoint(int gridX, int gridY){
		double pointX = GRID_LENGTH/2 + gridX*GRID_LENGTH; 
		double pointY = GRID_LENGTH/2 + gridY*GRID_LENGTH; 
		return new double[] {pointX,pointY};	
	}
	
	/**
	 * getter method for fieldMap
	 * @return fieldMap
	 */
	public List<Grid> getFieldMap(){
		return this.fieldMap;
	}
	
	
}

/**
 * Defines and keeps track of each grid square and its attributes
 */
class Grid {
	
	/**
	 * Define three statuses of a grid square: clear, blocked or null, for unchecked.
	 */
	enum Status {CLEAR,BLOCKED}	
	
	/**
	 * [0] for x, [1] for y axis of each grid square.
	 */
	private int[] gridIndex;
	private Status status;
	
	/**
	 * Empty constructor.
	 */
	public Grid(){}					
	
	/**
	 * Constructor for a grid square to hold location of this particular square on the field map
	 * @param gridX x-coordinate of grid square
	 * @param gridY y-coordinate of grid square
	 */
	public Grid(int gridX, int gridY)  {  
		gridIndex = new int[2];		
		gridIndex[0] = gridX;
		gridIndex[1] = gridY;
	}

	/**
	 * Getter method for status of grid square
	 * @return status of grid square
	 */
	public Status getStatus() {
		return status;
	}

	/**
	 * Setter method for status to whatever is input
	 * @param status input status to set grid square status to
	 */
	public void setStatus(Status status) {
		this.status = status;
	}
	
	/**
	 * Setter method to set status of a grid square to BLOCKED
	 */
	public void setBlocked(){
		this.status = Status.BLOCKED;
	}

	/**
	 * Getter method for gridIndex
	 * @return gridIndex
	 */
	public int[] getGridIndex() {
		return gridIndex;
	}
	
	/**
	 * Getter method for x-coordinate of gridIndex
	 * @return gridIndex[0] previously defined as x-coordinate of grid square
	 */
	public int getGridX(){
		return gridIndex[0];
	}
	
	/**
	 * Getter method for y-coordinate of gridIndex
	 * @return gridIndex[1] previously defined as y-coordinate of grid square
	 */
	public int getGridY(){
		return gridIndex[1];
	}
	
	
	/**
	 * Compares this instance of grid (specific grid square) with reference grid 
	 * by passing index of reference grid square.
	 * @param targetX x-coordinate of reference grid square
	 * @param targetY y-coordinate of reference grid square
	 * @return if equal T, if not F
	 */
	public boolean equals(int targetX, int targetY){
		if(gridIndex[0] == targetX &&  gridIndex[1] == targetY){
			return true;
		}else{
			return false;
		}
	}
}