package CompetitionExecution;

public class Grid {
	
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