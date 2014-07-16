import ilog.concert.IloColumn;
import ilog.concert.IloException;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.concert.IloObjective;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;
import input.Instance;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import colgen.InventoryPlan;
import colgen.Route;

public class solveMIRPTColgen {
	public static BufferedWriter bw = null;
	public static BufferedWriter solwriter = null;
	public static OutputStream logwriter = null;
	public static BufferedWriter mapwriter = null;

	public static void main(String[] args) {
		String inputfile = "D:\\The Folder\\IRP\\TestCases\\Problems";
		solveProblems(inputfile);
		System.out
				.println("________________Column Generation Done!!!________________");
	}

	static void solveProblems(String inputfile) {
		File file = new File(inputfile);
		if (file.isDirectory()) {
			File[] list = file.listFiles();
			for (int listIndex = 0; listIndex < list.length; listIndex++) {
				solveProblems(list[listIndex].getAbsolutePath());
			}
		} else {
			String fileName = "";
			if (file.isFile()) {
				fileName = file.getAbsolutePath();
				System.out.println("Attempting " + fileName + " ...");
				Instance instance = loadData.getInstance(fileName);

				boolean lp;
				boolean cuts;
				String slp;
				String scuts;
				if (fileName.contains("IP") || fileName.contains("ip")) {
					lp = false;
					slp = "IP-";
				} else {
					lp = true;
					slp = "LP-";
				}

				if (fileName.contains("with") || fileName.contains("WITH")) {
					cuts = true;
					scuts = "WithCuts-";
				} else {
					cuts = false;
					scuts = "NoCuts-";
				}

				try {
					fileName = file.getAbsolutePath();
					bw = new BufferedWriter(
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ fileName.substring(
													fileName.lastIndexOf("\\") + 1,
													fileName.indexOf("."))
											+ "-CG-"
											+ slp
											+ scuts
											+ (new SimpleDateFormat(
													"MMM-dd--HH-mm-ss"))
													.format(new Date())
											+ "-Formulation.txt"));
					solwriter = new BufferedWriter(
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ fileName.substring(
													fileName.lastIndexOf("\\") + 1,
													fileName.indexOf("."))
											+ "-CG-"
											+ slp
											+ scuts
											+ (new SimpleDateFormat(
													"MMM-dd--HH-mm-ss"))
													.format(new Date())
											+ "-Solution.txt"));
					logwriter = new FileOutputStream(
							getTargetLoc(fileName)
									+ "\\"
									+ fileName.substring(
											fileName.lastIndexOf("\\") + 1,
											fileName.indexOf("."))
									+ "-CG-"
									+ slp
									+ scuts
									+ (new SimpleDateFormat("MMM-dd--HH-mm-ss"))
											.format(new Date()) + "-Log.txt");

					// mapwriter = new BufferedWriter(
					// new FileWriter(
					// getTargetLoc(fileName)
					// + "\\"
					// + fileName.substring(
					// fileName.lastIndexOf("\\") + 1,
					// fileName.indexOf("."))
					// + "-CG-"
					// + slp
					// + scuts
					// + (new SimpleDateFormat(
					// "MMM-dd--HH-mm-ss"))
					// .format(new Date())
					// + "-Map.tex"));
				} catch (IOException e) {
					e.printStackTrace();
				}

				processProblem(instance, lp, cuts, fileName);

				try {
					solwriter.close();
					bw.close();
					// mapwriter.write("\\end{document}");
					// mapwriter.close();
					logwriter.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}

	private static void processProblem(Instance instance, boolean lp,
			boolean cuts, String fileName) {
		try {
			IloCplex cplex = new IloCplex();

			// PLANS
			ArrayList<Route>[][] routePlans = createRoutePlan(instance);
			ArrayList<InventoryPlan> invPlans = new ArrayList<InventoryPlan>();

			// VARIABLES
			ArrayList<IloNumVar>[][] routeVars = createRouteVariables(instance);
			ArrayList<IloNumVar> invVars = new ArrayList<IloNumVar>();

			// CONSTRAINTS
			IloRange[][][] qtyRouteLinkCons = createQtyRouteLinkConstraints(
					cplex, instance);
			IloRange[][] routeConvCons = createRouteConvexityConstraints(cplex,
					instance);
			IloRange invConvCons = createInvConvexityConstraints(cplex,
					instance);

			// OBJECTIVE
			IloObjective obj = cplex.addMinimize();

			double[][][] linkDuals = new double[instance.getTime()][instance
					.getNoNodes()][instance.getNoNodes()];
			double[][] routeXDuals = new double[instance.getTime()][instance
					.getNoNodes()];
			double invXDual = 0;

			// TODO PUT INITIAL SOLUTION

			int iteration = 0;

			while (true) {
				boolean oneMoreIteration = false;
				for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
					for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
						Route r = new Route(linkDuals[timeIndex], nodeIndex,
								instance);
						System.out.println(r.reducedCost + ":<--RouteRCost");
						if ((r.reducedCost - routeXDuals[timeIndex][nodeIndex]) < 0) {
							oneMoreIteration = true;
							routePlans[timeIndex][nodeIndex].add(r);
							routeVars[timeIndex][nodeIndex]
									.add(cplex
											.numVar(getRouteColumn(
													cplex,
													instance,
													obj,
													qtyRouteLinkCons[timeIndex],
													routeConvCons[timeIndex][nodeIndex],
													r, nodeIndex),
													0,
													1,
													IloNumVarType.Float,
													"lambda_"
															+ routePlans[timeIndex][nodeIndex]
																	.size()
															+ "^{" + timeIndex
															+ "," + nodeIndex
															+ "}"));
						}
					}
				}
				InventoryPlan i = new InventoryPlan(linkDuals, instance);
				System.out.println(i.reducedCost + "<--InvRcost");
				if ((i.reducedCost - invXDual) < 0) {
					oneMoreIteration = true;
					invPlans.add(i);
					invVars.add(cplex.numVar(
							getInvColumn(cplex, instance, obj,
									qtyRouteLinkCons, invConvCons, i), 0, 1,
							IloNumVarType.Float, "mu_" + invPlans.size()));
				}

				if (!oneMoreIteration) {
					break;
				}
				System.out
						.println("-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-");
				System.out.println("TRYING MAIN PROBLEM");
				System.out
						.println("-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-");
				for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
					for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
						for (int nodeIndex2 = 0; nodeIndex2 < instance
								.getNoNodes(); nodeIndex2++) {
							System.out
									.println(qtyRouteLinkCons[timeIndex][nodeIndex][nodeIndex2]);
						}
						System.out.println(routeConvCons[timeIndex][nodeIndex]);
					}
				}
				System.out.println(invConvCons);

				try {
					cplex.setOut(new FileOutputStream("E:\\MainOutput.txt"));
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				}

				if (cplex.solve()) {
					System.out.println("Solved Iteration :" + ++iteration);
					System.out.println("Objective:" + cplex.getObjValue());
					getduals(cplex, instance, qtyRouteLinkCons, linkDuals,
							routeConvCons, routeXDuals, invConvCons, invXDual);
				} else {
					System.err
							.println("Couldn't solve Main Problem at iteration:"
									+ ++iteration);
					System.err.println(cplex.getStatus());
				}
			}

		} catch (IloException e) {
			e.printStackTrace();
		}

	}

	private static void getduals(IloCplex cplex, Instance instance,
			IloRange[][][] qtyRouteLinkCons, double[][][] linkDuals,
			IloRange[][] routeConvCons, double[][] routeXDuals,
			IloRange invConvCons, double invXDual)
			throws UnknownObjectException, IloException {
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					linkDuals[timeIndex][fromNodeIndex][toNodeIndex] = cplex
							.getDual(qtyRouteLinkCons[timeIndex][fromNodeIndex][toNodeIndex]);
					System.out.print("linkDuals[" + timeIndex + "]["
							+ fromNodeIndex + "][" + toNodeIndex + "]:"
							+ linkDuals[timeIndex][fromNodeIndex][toNodeIndex]
							+ "; ");
				}
				routeXDuals[timeIndex][fromNodeIndex] = cplex
						.getDual(routeConvCons[timeIndex][fromNodeIndex]);
				System.out.println("routeXDuals[" + timeIndex + "]["
						+ fromNodeIndex + "]:"
						+ routeXDuals[timeIndex][fromNodeIndex] + "; ");
			}
		}
		invXDual = cplex.getDual(invConvCons);
		System.out.println(invXDual);
	}

	private static IloColumn getInvColumn(IloCplex cplex, Instance instance,
			IloObjective obj, IloRange[][][] qtyRouteLinkCons,
			IloRange invConvCons, InventoryPlan ip) throws IloException {
		IloColumn col = cplex.column(obj, ip.planCost);
		System.out.println("In Inventory Plan");
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					col = col
							.and(cplex
									.column(qtyRouteLinkCons[timeIndex][fromNodeIndex][toNodeIndex],
											-ip.qtyTran[timeIndex][fromNodeIndex][toNodeIndex]));
				}
			}
		}
		col = col.and(cplex.column(invConvCons, 1));
		return col;
	}

	private static IloColumn getRouteColumn(IloCplex cplex, Instance instance,
			IloObjective obj, IloRange[][] linkCons, IloRange convCons,
			Route r, int startNode) throws IloException {
		System.out.println("In Route Column generation");
		IloColumn col = cplex.column(obj, r.routeCost);
		for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
			for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
				col = col
						.and(cplex
								.column(linkCons[fromIndex][toIndex],
										(instance.getVehicle(startNode)
												.getVehicleCapacity() * r.route[fromIndex][toIndex])));
			}
		}
		col = col.and(cplex.column(convCons, 1));
		return col;
	}

	private static IloRange createInvConvexityConstraints(IloCplex cplex,
			Instance instance) throws IloException {
		IloRange cons = cplex.addRange(1, 1, "IX");
		return cons;
	}

	private static IloRange[][] createRouteConvexityConstraints(IloCplex cplex,
			Instance instance) throws IloException {
		IloRange[][] cons = new IloRange[instance.getTime()][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			cons[timeIndex] = new IloRange[instance.getNoNodes()];
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				cons[timeIndex][nodeIndex] = cplex.addRange(1, 1, "RX^{"
						+ timeIndex + "," + nodeIndex + "}");
			}
		}
		return cons;
	}

	private static IloRange[][][] createQtyRouteLinkConstraints(IloCplex cplex,
			Instance instance) throws IloException {
		IloRange[][][] cons = new IloRange[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			cons[timeIndex] = new IloRange[instance.getNoNodes()][];
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				cons[timeIndex][fromNodeIndex] = new IloRange[instance
						.getNoNodes()];
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					cons[timeIndex][fromNodeIndex][toNodeIndex] = cplex
							.addRange(0, Double.MAX_VALUE, "QRL^" + timeIndex
									+ "_{" + fromNodeIndex + "," + toNodeIndex
									+ "}");
				}
			}
		}
		return cons;
	}

	private static ArrayList<Route>[][] createRoutePlan(Instance instance) {
		ArrayList<Route>[][] list = new ArrayList[instance.getTime()][instance
				.getNoNodes()];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				list[timeIndex][nodeIndex] = new ArrayList<Route>();
			}
		}
		return list;
	}

	private static ArrayList<IloNumVar>[][] createRouteVariables(
			Instance instance) {
		ArrayList<IloNumVar>[][] list = new ArrayList[instance.getTime()][instance
				.getNoNodes()];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				list[timeIndex][nodeIndex] = new ArrayList<IloNumVar>();
			}
		}
		return list;
	}

	private static String getTargetLoc(String fileName) {
		String lastfolder = fileName.substring(0, fileName.lastIndexOf("."));
		lastfolder = lastfolder.replace("TestCases",
				"TestCases\\ColumnGeneration\\");
		lastfolder = lastfolder.replace("\\Problems", "\\");
		File file = new File(lastfolder);
		if (!file.exists()) {
			if (file.mkdirs()) {
				System.out.println("Required directories created.....");
			} else {
				return null;
			}
		}
		return file.getAbsolutePath();
	}
}
