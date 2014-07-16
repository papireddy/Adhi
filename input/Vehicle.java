package input;
public class Vehicle {

	int baseLocation;
	float perUnitCost;
	float vehicleCapacity;
	float maxDistance;

	public Vehicle(int baseLocation, float perUnitCost, float vehicleCapacity,
			float maxDistance) {
		super();
		this.baseLocation = baseLocation;
		this.perUnitCost = perUnitCost;
		this.vehicleCapacity = vehicleCapacity;
		this.maxDistance = maxDistance;
	}

	public int getBaseLocation() {
		return baseLocation;
	}

	public float getPerUnitCost() {
		return perUnitCost;
	}

	public double getVehicleCapacity() {
		return vehicleCapacity;
	}

	public float getMaxDistance() {
		return maxDistance;
	}

}
