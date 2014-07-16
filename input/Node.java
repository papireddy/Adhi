package input;
public class Node {

	int id;
	float xCoordinate;
	float yCoordinate;
	double minCapacity;
	double maxCapacity;
	float[] demand;
	float startInv;
	float holdingCost;

	public float getHoldingCost() {
		return holdingCost;
	}

	public void setHoldingCost(float holdingCost) {
		this.holdingCost = holdingCost;
	}

	public Node(int id, double minCapacity, double maxCapacity, float[] demand) {
		super();
		this.id = id;
		this.minCapacity = minCapacity;
		this.maxCapacity = maxCapacity;
		this.demand = demand;
	}

	public float getStartInv() {
		return startInv;
	}

	public void setStartInv(float startInv) {
		this.startInv = startInv;
	}

	public float getxCoordinate() {
		return xCoordinate;
	}

	public void setxCoordinate(float xCoordinate) {
		this.xCoordinate = xCoordinate;
	}

	public float getyCoordinate() {
		return yCoordinate;
	}

	public void setyCoordinate(float yCoordinate) {
		this.yCoordinate = yCoordinate;
	}

	public int getId() {
		return id;
	}

	public double getMinCapacity() {
		return minCapacity;
	}

	public double getMaxCapacity() {
		return maxCapacity;
	}

	public float[] getDemand() {
		return demand;
	}

	public void setDemands(float[] demand) {
		this.demand = demand;
	}

}
