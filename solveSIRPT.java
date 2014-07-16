import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.cplex.IloCplex;
import input.Instance;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class solveSIRPT {

	public static BufferedWriter bw = null;
	public static BufferedWriter solwriter = null;
	public static OutputStream logwriter = null;

	public static void main(String[] args) {
		String inputfile = "D:\\The Folder\\TestCases\\Problems";
		try {
			BufferedWriter activitywriter = new BufferedWriter(new FileWriter(
					"D:\\The Folder\\TestCases\\Activity\\activity"
							+ (new SimpleDateFormat("MMM-dd--HH-mm-ss"))
									.format(new Date()) + ".txt"));
			activitywriter.write("Started at :"
					+ (new SimpleDateFormat("MMM-dd HH:mm:ss"))
							.format(new Date()));
			activitywriter.newLine();
			activitywriter.flush();
			solveProblems(inputfile);
			activitywriter.write("Ended at :"
					+ (new SimpleDateFormat("MMM-dd HH:mm:ss"))
							.format(new Date()));
			activitywriter.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println("DONE with all the problems!!!");
	}

	private static void solveProblems(String inputfile) {
		File file = new File(inputfile);
		if (file.isDirectory()) {
			File[] list = file.listFiles();
			for (int listIndex = 0; listIndex < list.length; listIndex++) {
				solveProblems(list[listIndex].getAbsolutePath());
			}
		} else {
			String fileName = "";
			long startTime;
			long endTime;

			if (file.isFile()) {
				try {
					fileName = file.getAbsolutePath();
					bw = new BufferedWriter(
							new FileWriter(getTargetLoc(fileName,
									"Formulations")
									+ "\\"
									+ fileName.substring(
											fileName.lastIndexOf("\\") + 1,
											fileName.indexOf("."))
									+ "-Formulation.txt"));
					solwriter = new BufferedWriter(new FileWriter(getTargetLoc(
							fileName, "Solutions")
							+ "\\"
							+ fileName.substring(
									fileName.lastIndexOf("\\") + 1,
									fileName.indexOf(".")) + "-solution.txt"));
					logwriter = new FileOutputStream(getTargetLoc(fileName,
							"Logs")
							+ "\\"
							+ fileName.substring(
									fileName.lastIndexOf("\\") + 1,
									fileName.indexOf(".")) + "-log.txt");
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			Instance instance = loadData.getInstance(fileName);

			processTheProblem(instance);
			try {
				solwriter.close();
				bw.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private static String getTargetLoc(String fileName, String loc) {
		String lastfolder = fileName.substring(0, fileName.lastIndexOf("\\"));
		lastfolder = lastfolder.replace("TestCases", "TestCases\\" + loc);
		lastfolder = lastfolder.replace("\\Problems", "\\");
		File file = new File(lastfolder);
		if (!file.exists()) {
			if (file.mkdirs()) {
				// System.out.println("Necessary folders for " + loc
				// + " created..........");
			} else {
				return null;
			}
		}
		return file.getAbsolutePath();
	}

	private static void processTheProblem(Instance instance) {
		try {
			IloCplex cplex = new IloCplex();

			IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
			IloNumVar[][][] rouVar = createRouteVariables(cplex, instance);
			IloNumVar[][][] qtyVar = createQuantityVariables(cplex, instance);
			IloNumVar[][] nextQtyVar = createNextQtyVar(cplex, instance);

			createConstraints(cplex, instance, invVar, rouVar, qtyVar,
					nextQtyVar);

			createObjective(cplex, instance, rouVar, invVar);
			bw.flush();
			bw.close();
			if (cplex.solve()) {
				System.out.println("solved:" + cplex.getObjValue());
				getSolution(cplex, instance, qtyVar, rouVar, invVar, nextQtyVar);
			} else {
				System.out.println("Can't SOLVE!!!");
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void getSolution(IloCplex cplex, Instance instance,
			IloNumVar[][][] qtyVar, IloNumVar[][][] rouVar,
			IloNumVar[][] invVar, IloNumVar[][] nextVar) {
		try {
			solwriter.write("Objective Value:" + cplex.getObjValue());
			solwriter.newLine();

			solwriter.write("Inventory Variables:");
			solwriter.newLine();
			for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
				for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					solwriter.write(invVar[timeIndex][nodeIndex].getName()
							+ ":"
							+ cplex.getValue(invVar[timeIndex][nodeIndex]));
					solwriter.write("\t");
					solwriter.flush();
				}
				solwriter.newLine();
			}

			solwriter.write("Quantity Variables:");
			solwriter.newLine();
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (cplex
								.getValue(qtyVar[timeIndex][fromNodeIndex][toNodeIndex]) != 0) {
							solwriter
									.write(qtyVar[timeIndex][fromNodeIndex][toNodeIndex]
											.getName()
											+ ":"
											+ cplex.getValue(qtyVar[timeIndex][fromNodeIndex][toNodeIndex])
											+ ";");
						}
					}
					solwriter.write("\t\t\t");
				}
				solwriter.newLine();
			}

			solwriter.write("Route Variables:");
			solwriter.newLine();
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (cplex
								.getValue(rouVar[timeIndex][fromNodeIndex][toNodeIndex]) != 0)
							solwriter
									.write(rouVar[timeIndex][fromNodeIndex][toNodeIndex]
											.getName()
											+ ":"
											+ cplex.getValue(rouVar[timeIndex][fromNodeIndex][toNodeIndex])
											+ "; ");
					}
					solwriter.write("\t\t\t");
				}
				solwriter.newLine();
			}

			solwriter.write("Next Variables:");
			solwriter.newLine();
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					solwriter.write(nextVar[timeIndex][nodeIndex].getName()
							+ ":"
							+ cplex.getValue(nextVar[timeIndex][nodeIndex]));
					solwriter.write("\t");
					solwriter.flush();
				}
				solwriter.newLine();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void createObjective(IloCplex cplex, Instance instance,
			IloNumVar[][][] rouVar, IloNumVar[][] invVar) {

		try {
			bw.write("Objective:");
			bw.newLine();
			bw.write(cplex.addMinimize(cplex.sum(
					getHoldingCost(cplex, instance, invVar),
					getTransCost(cplex, instance, rouVar)), "Objective")
					+ "");
			bw.newLine();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static IloNumExpr getTransCost(IloCplex cplex, Instance instance,
			IloNumVar[][][] routeVar) throws IloException, IOException {
		IloNumExpr expr = cplex.constant(0);
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
				for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
					expr = cplex
							.sum(expr,
									cplex.prod(
											(instance.getVehicle(0)
													.getPerUnitCost() * instance.distanceMatrix[fromIndex][toIndex]),
											routeVar[timeIndex][fromIndex][toIndex]));
				}
			}
		}
		return expr;
	}

	private static IloNumExpr getHoldingCost(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar) throws IloException, IOException {

		IloNumExpr expr = cplex.constant(0);
		for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				expr = cplex.sum(expr, cplex.prod(instance.getNode(nodeIndex)
						.getHoldingCost(), invVar[timeIndex][nodeIndex]));
			}
		}
		return expr;
	}

	private static void createConstraints(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar, IloNumVar[][][] rouVar,
			IloNumVar[][][] qtyVar, IloNumVar[][] nextQtyVar) {
		try {
			createInvBalConst(cplex, instance, invVar, qtyVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createRouteBalConstraints(cplex, instance, rouVar);
			createCapCon(cplex, instance, qtyVar, rouVar);
			createSubTourEliminationConstraints(cplex, instance, nextQtyVar,
					rouVar);

		} catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	private static void createSubTourEliminationConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][] nextQtyVar, IloNumVar[][][] rouVar)
			throws IloException, IOException {
		bw.write("Sub-tour Elimination Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int focusNodeIndex = 1; focusNodeIndex < instance.getNoNodes(); focusNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					bw.write(cplex.addLe(
							cplex.sum(
									nextQtyVar[timeIndex][focusNodeIndex],
									cplex.prod(-1,
											nextQtyVar[timeIndex][toNodeIndex]),
									cplex.prod(
											instance.getVehicle(0)
													.getVehicleCapacity(),
											rouVar[timeIndex][focusNodeIndex][toNodeIndex])),
							instance.getVehicle(0).getVehicleCapacity()
									- instance.getNode(toNodeIndex).getDemand()[timeIndex])
							+ "\n");
				}
			}
		}
	}

	private static void createCapCon(IloCplex cplex, Instance instance,
			IloNumVar[][][] qtyVar, IloNumVar[][][] routeVar)
			throws IloException, IOException {
		bw.write("Quantity Capcity Link Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
				for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
					bw.write(cplex.addLe(qtyVar[timeIndex][fromIndex][toIndex],
							cplex.prod(instance.getVehicle(0)
									.getVehicleCapacity(),
									routeVar[timeIndex][fromIndex][toIndex]))
							+ "");
					bw.newLine();
				}
			}
		}
	}

	private static void createRouteBalConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][] routeVar) throws IloException,
			IOException {
		bw.write("Route Balance Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {

			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr inExpr = cplex.constant(0);
				IloNumExpr outExpr = cplex.constant(0);
				for (int otherIndex = 0; otherIndex < instance.getNoNodes(); otherIndex++) {
					inExpr = cplex.sum(inExpr,
							routeVar[timeIndex][otherIndex][nodeIndex]);
					outExpr = cplex.sum(outExpr,
							routeVar[timeIndex][nodeIndex][otherIndex]);
				}
				bw.write(cplex.addEq(inExpr, outExpr) + "");
				bw.newLine();
			}
		}

	}

	private static void createMaxInvConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][] invVar) throws IloException,
			IOException {
		bw.write("Maximum Inventory Constraints:");
		bw.newLine();
		for (int timeIndex = 1; timeIndex < instance.getTime() + 1; timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				bw.write(cplex.addGe(instance.getNode(nodeIndex)
						.getMaxCapacity(), invVar[timeIndex][nodeIndex])
						+ "");
				bw.newLine();
			}
		}
	}
	private static void createMinInvConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][] invVar) throws IloException,
			IOException {
		bw.write("Minimum Inventory Constraints:");
		bw.newLine();
		for (int timeIndex = 1; timeIndex < instance.getTime() + 1; timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				bw.write(cplex.addLe(instance.getNode(nodeIndex)
						.getMinCapacity(), invVar[timeIndex][nodeIndex])
						+ "");
				bw.newLine();
			}
		}
	}

	private static void createInvBalConst(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar, IloNumVar[][][] qtyVar) throws IloException,
			IOException {
		bw.write("Inventory Balance Constraints:\n");
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int focusNodeIndex = 0; focusNodeIndex < instance.getNoNodes(); focusNodeIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int otherNodeIndex = 0; otherNodeIndex < instance
						.getNoNodes(); otherNodeIndex++) {
					expr = cplex
							.sum(expr,
									qtyVar[timeIndex][otherNodeIndex][focusNodeIndex],
									cplex.prod(
											-1,
											qtyVar[timeIndex][focusNodeIndex][otherNodeIndex]));
				}
				bw.write(cplex.addEq(
						invVar[timeIndex + 1][focusNodeIndex],
						cplex.sum(
								invVar[timeIndex][focusNodeIndex],
								expr,
								cplex.constant(-instance
										.getNode(focusNodeIndex).getDemand()[timeIndex])))
						+ "\n");
			}
		}
	}

	private static IloNumVar[][] createNextQtyVar(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar[][] nextQtyVar = new IloNumVar[instance.getTime()][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			nextQtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()];
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				nextQtyVar[timeIndex][nodeIndex] = cplex.numVar(0,
						Float.MAX_VALUE, IloNumVarType.Float, "u^{" + timeIndex
								+ "}_" + nodeIndex);
				bw.write(nextQtyVar[timeIndex][nodeIndex] + "\t");
			}
			bw.newLine();
		}
		return nextQtyVar;
	}

	private static IloNumVar[][][] createQuantityVariables(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar qtyVar[][][] = new IloNumVar[instance.getTime() + 1][][];
		for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
			qtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				qtyVar[timeIndex][nodeIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int otherNodeIndex = 0; otherNodeIndex < instance
						.getNoNodes(); otherNodeIndex++) {
					qtyVar[timeIndex][nodeIndex][otherNodeIndex] = cplex
							.numVar(0, Double.MAX_VALUE, IloNumVarType.Float,
									"q^" + timeIndex + "_{" + nodeIndex + ","
											+ otherNodeIndex + "}");

				}
			}
		}
		return qtyVar;
	}

	private static IloNumVar[][][] createRouteVariables(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar[][][] rouVar = new IloNumVar[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < rouVar.length; timeIndex++) {
			rouVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];
			for (int fromIndex = 0; fromIndex < rouVar[timeIndex].length; fromIndex++) {
				rouVar[timeIndex][fromIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int toIndex = 0; toIndex < rouVar[timeIndex][fromIndex].length; toIndex++) {

					rouVar[timeIndex][fromIndex][toIndex] = cplex.numVar(0, 1,
							IloNumVarType.Bool, "y^{" + timeIndex + "}" + "_{"
									+ fromIndex + "," + toIndex + "}");

				}
			}
		}
		return rouVar;
	}

	private static IloNumVar[][] createInventoryVariables(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar[][] invVar = new IloNumVar[instance.getTime() + 1][];
		for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
			invVar[timeIndex] = new IloNumVar[instance.getNoNodes()];
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				if (timeIndex == 0) {
					invVar[timeIndex][nodeIndex] = cplex.numVar(instance
							.getNode(nodeIndex).getStartInv(), instance
							.getNode(nodeIndex).getStartInv(), "I^" + timeIndex
							+ "_" + nodeIndex);
				} else {
					invVar[timeIndex][nodeIndex] = cplex.numVar(0,
							Double.MAX_VALUE, IloNumVarType.Float, "I^"
									+ timeIndex + "_" + nodeIndex);
				}
			}
		}
		return invVar;
	}

}
