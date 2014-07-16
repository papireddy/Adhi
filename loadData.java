import input.Instance;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class loadData {
	public static void main(String[] args) {
		System.out.println("Give File Name!!");
	}
	public static Instance getInstance(String fileName) {
		Instance ins = null;
		try {
			BufferedReader br = new BufferedReader(new FileReader(fileName));
			String line;
			String[] lines = null;
			int noNodes = 0;
			long nodeMinCap = 0;
			long nodeMaxCap = 0;

			long vehicleMaxCap = 0;
			long vehicleMaxDistance = 0;

			float unitCostless = 0;
			float unitCostmore = 0;
			float[] demands = null;

			float maxDistance = 0;
			float maxDemand = 0;

			while ((line = br.readLine()) != null) {

				if (line.contains("Node Capacity")) {
					line = br.readLine();
					lines = line.split(" ");
					nodeMinCap = Long.valueOf(lines[0]);
					nodeMaxCap = Long.valueOf(lines[1]);

				} else if (line.contains("Travel Cost")) {
					line = br.readLine();
					lines = line.split(" ");
					unitCostless = Float.valueOf(lines[0]);
					unitCostmore = Float.valueOf(lines[1]);

				} else if (line.contains("Vehicle Capacity")) {
					line = br.readLine();
					vehicleMaxCap = Long.valueOf(line);

				} else if (line.contains("Vehicle Maximum Distance")) {
					line = br.readLine();
					vehicleMaxDistance = Long.valueOf(line);

				} else if (line.contains("Distance Matrix")) {
					line = br.readLine();
					if (line.contains(" ")) {
						lines = line.split(" ");
					} else if (line.contains("\t")) {
						lines = line.split("\t");
					}
					noNodes = lines.length;
					ins = new Instance(noNodes);
					for (int lineIndex = 0; lineIndex < lines.length; lineIndex++) {
						for (int partIndex = 0; partIndex < lines.length; partIndex++) {
							ins.distanceMatrix[lineIndex][partIndex] = Float
									.valueOf(lines[partIndex]);
							// finding max distance for limit

							maxDistance = maxDistance
									+ ins.distanceMatrix[lineIndex][partIndex];

						}
						if (lineIndex < lines.length - 1) {
							line = br.readLine();
							if (line.contains(" ")) {
								lines = line.split(" ");
							} else if (line.contains("\t")) {
								lines = line.split("\t");
							}
						}
					}

					for (int nodeIndex = 1; nodeIndex < lines.length; nodeIndex++) {
						ins.addVehicle(nodeIndex, unitCostless, vehicleMaxCap,
								vehicleMaxDistance);
					}

				} else if (line.contains("Demands")) {
					int time = 0;

					for (int lineIndex = 1; lineIndex < noNodes; lineIndex++) {

						line = br.readLine();
						lines = line.split(" ");
						time = lines.length;

						demands = new float[lines.length];
						for (int timeIndex = 0; timeIndex < demands.length; timeIndex++) {
							demands[timeIndex] = Float
									.valueOf(lines[timeIndex]);

							maxDemand = maxDemand
									+ Math.abs(demands[timeIndex]);

						}
						ins.addNode(lineIndex, nodeMinCap, nodeMaxCap, demands);
					}
					ins.addHub(0, 0, maxDemand * ins.getNoNodes() * time * 200,
							new float[time]);

				} else if (line.contains("Start Inventory")) {

					for (int nodeIndex = 1; nodeIndex < noNodes; nodeIndex++) {
						line = br.readLine();
						ins.setStartInv(nodeIndex, Float.valueOf(line));
					}
				} else if (line.contains("Holding Cost")) {
					line = br.readLine();
					lines = line.split(" ");
					for (int holdingIndex = 0; holdingIndex < lines.length; holdingIndex++) {
						ins.getNode(holdingIndex + 1).setHoldingCost(
								Float.valueOf(lines[holdingIndex]));
					}
				}
			}

			ins.addHubVehicle(unitCostmore, maxDemand, maxDistance);

			ins.setStartInv(0, maxDemand * ins.getNoNodes() * ins.getTime() * 2);

		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return ins;
	}
}
