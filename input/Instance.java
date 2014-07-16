package input;

import java.util.ArrayList;

public class Instance {

	public float[][] distanceMatrix;
	ArrayList<Node> nodeList = new ArrayList<Node>();
	ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();

	public int getTime() {
		return nodeList.get(1).demand.length;
	}

	public int getNoNodes() {
		return nodeList.size();
	}

	public Node getNode(int index) {
		return nodeList.get(index);
	}

	public Instance(int noNodes) {
		distanceMatrix = new float[noNodes][noNodes];
	}

	public void addHubVehicle(float unitCostmore, float capacity, float distance) {
		Vehicle v = new Vehicle(0, unitCostmore, capacity, distance);
		vehicleList.add(0, v);
	}

	public void addVehicle(int nodeIndex, float unitCostless,
			float vehicleMaxCap, float vehicleMaxDistance) {
		Vehicle v = new Vehicle(nodeIndex, unitCostless, vehicleMaxCap,
				vehicleMaxDistance);
		vehicleList.add(v);
	}

	public void addHub(int id, long minCap, float maxValue, float[] demands) {
		Node n = new Node(id, minCap, maxValue, demands);
		nodeList.add(0, n);
	}

	public void addHubDemands(float[] demand) {
		nodeList.get(0).setDemands(demand);
	}
	public void addNode(int id, long minCap, long maxCap, float[] demands) {
		Node n = new Node(id, minCap, maxCap, demands);
		nodeList.add(n);
	}

	public void setStartInv(int nodeIndex, float startInv) {
		nodeList.get(nodeIndex).setStartInv(startInv);
	}

	public Vehicle getVehicle(int vehicleIndex) {
		return vehicleList.get(vehicleIndex);
	}

	public void setHoldingCost(int nodeIndex, Float holdingCost) {
		nodeList.get(nodeIndex).setHoldingCost(holdingCost);
	}
}
