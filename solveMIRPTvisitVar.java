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

public class solveMIRPTvisitVar {

	public static BufferedWriter bw = null;
	public static BufferedWriter solwriter = null;
	public static OutputStream logwriter = null;

	public static void main(String[] args) {
		String inputfile = "D:\\The Folder\\IRP\\TestCases\\Problems";
		solveProblems(inputfile);
		System.out
				.println("________________Visit Formulation Done!!!________________");
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
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ (new SimpleDateFormat(
													"MMM-dd--HH-mm-ss"))
													.format(new Date())
											+ "-Formulation.txt"));
					solwriter = new BufferedWriter(
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ (new SimpleDateFormat(
													"MMM-dd--HH-mm-ss"))
													.format(new Date())
											+ "-Solution.txt"));
					logwriter = new FileOutputStream(
							getTargetLoc(fileName)
									+ "\\"
									+ (new SimpleDateFormat("MMM-dd--HH-mm-ss"))
											.format(new Date()) + "-Log.txt");
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			System.out.println("Attempting " + fileName + " ...");
			Instance instance = loadData.getInstance(fileName);
			boolean lp;
			boolean cuts;
			if (fileName.contains("ip") || fileName.contains("IP")) {
				lp = false;
			} else {
				lp = true;
			}

			if (fileName.contains("with") || fileName.contains("WITH")) {
				cuts = true;
			} else {
				cuts = false;
			}

			processProblem(instance, lp, cuts, fileName);

			try {
				solwriter.close();
				bw.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private static void processProblem(Instance instance, boolean lp,
			boolean cuts, String fileName) {
		try {
			IloCplex cplex = new IloCplex();

			IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
			IloNumVar[][][] qtyVar = createQuantityVariables(cplex, instance);
			IloNumVar[][][][] routeVar = null;
			IloNumVar[][][][] visitVar = null;
			IloNumVar[][][] nextVar = createNextVaribles(cplex, instance);

			if (lp) {
				routeVar = createRouteVariablesLP(cplex, instance, "y");
				visitVar = createRouteVariablesLP(cplex, instance, "z");
			} else {
				routeVar = createRouteVariablesIP(cplex, instance, "y");
				visitVar = createRouteVariablesIP(cplex, instance, "z");
			}

			createConstraints(cplex, instance, cuts, invVar, qtyVar, routeVar,
					visitVar, nextVar);

			createObjective(cplex, instance, routeVar, invVar);

			setParameters(cplex);
			long startTime = System.nanoTime();
			if (cplex.solve()) {
				System.out.println(fileName.substring(fileName
						.lastIndexOf("\\") + 1)
						+ "-Solved-"
						+ cplex.getObjValue()
						+ "- in Time-"
						+ ((System.nanoTime() - startTime) / 1E9) + " sec");
				getSolution(cplex, instance, invVar, qtyVar, routeVar,
						visitVar, nextVar);
			} else {
				System.out.println("Didn't Solve!!!" + cplex.getStatus());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

	}
	private static void createObjective(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][] invVar) {

		try {
			bw.write("Objective:");
			bw.newLine();
			bw.write(cplex.addMinimize(cplex.sum(
					getHoldingCost(cplex, instance, invVar),
					getTransCost(cplex, instance, routeVar)), "Objective")
					+ "");
			bw.newLine();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static IloNumExpr getTransCost(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		IloNumExpr expr = cplex.constant(0);
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
					for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
						expr = cplex
								.sum(expr,
										cplex.prod(
												(instance.getVehicle(
														vehicleIndex)
														.getPerUnitCost() * instance.distanceMatrix[fromIndex][toIndex]),
												routeVar[timeIndex][vehicleIndex][fromIndex][toIndex]));
					}
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
			boolean cuts, IloNumVar[][] invVar, IloNumVar[][][] qtyVar,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar,
			IloNumVar[][][] nextVar) {
		try {
			createInvBalance(instance, cplex, qtyVar, invVar);
			createRouteBalConstraints(cplex, instance, routeVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createvisitQtyLinkConstraints(cplex, instance, qtyVar, visitVar);
			createvisitRouteInLinkConstraints(cplex, instance, visitVar,
					routeVar);
			createvisitRouteOutLinkConstraints(cplex, instance, visitVar,
					routeVar);

			// createOneVehiclePerNodeConstraints(cplex, instance, routeVar);
			createVehicleStartNodeConstraints(cplex, instance, routeVar);
			createMaxDistanceConstraints(cplex, instance, routeVar);

			createVehicleReturnConstraints(cplex, instance, visitVar, routeVar);
			createSameNodeRouteVarZero(cplex, instance, visitVar, routeVar);
			// createSubTourElimination(cplex, instance, routeVar, nextVar);

			if (cuts) {
				// Min Visit Cuts
				createMinNumberVisitCuts(cplex, instance, routeVar);

				createDemandInvDiffCuts(cplex, instance, routeVar, invVar);

				// IDD cuts
				createStartInvDemDiffCuts(cplex, instance, routeVar);

				// eq15 Cuts
				// createEq15Cuts(cplex, instance, routeVar, invVar);

				// eq14 Cuts
				createEq14Cuts(cplex, instance, routeVar, invVar);

				// Vehicle loop visit cuts
				// createLoopVisitCuts(cplex, instance, routeVar, visitVar);
				createVisitRouteCuts(cplex, instance, routeVar, visitVar);
			}

		} catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static void createVisitRouteCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar)
			throws IloException, IOException {
		bw.write("Route Visit Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						bw.write(cplex
								.addGe(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
								+ "\t");
					}
				}
				bw.newLine();
			}
		}
	}

	private static void createSubTourElimination(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar,
			IloNumVar[][][] nextVar) throws IloException, IOException {
		bw.write("SubTour Elimination Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex) {
							continue;
						}
						bw.write(cplex.addLe(
								cplex.sum(
										nextVar[timeIndex][vehicleIndex][fromNodeIndex],
										cplex.prod(
												-1,
												nextVar[timeIndex][vehicleIndex][toNodeIndex]),
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])),
								instance.getVehicle(vehicleIndex)
										.getVehicleCapacity()
										- instance.getNode(fromNodeIndex)
												.getDemand()[timeIndex])
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createSameNodeRouteVarZero(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Force From a node To same node variable to zero:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex) {
							// bw.write(cplex
							// .addEq(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
							// 0)
							// + "\t");
							bw.write(cplex
									.addEq(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
											0)
									+ "\t");
						}
					}
					bw.newLine();
				}
			}
		}
	}

	private static void createLoopVisitCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar)
			throws IloException, IOException {
		bw.write("Loop Visit Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr rouExpr = cplex.constant(0);
				IloNumExpr visExpr = cplex.constant(0);
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < visitVar.length; toNodeIndex++) {
						rouExpr = cplex
								.sum(rouExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
						visExpr = cplex
								.sum(visExpr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
					}
				}
				bw.write(cplex.addLe(rouExpr, cplex.sum(visExpr, 1)) + "");
				bw.newLine();
			}
		}
	}

	private static void createEq14Cuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][] invVar)
			throws IloException, IOException {
		bw.write("Inventory Demand multi period Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr expr = cplex.constant(1);
				double demandSum = 0;
				for (int toTimeIndex = timeIndex; toTimeIndex < instance
						.getTime(); toTimeIndex++) {
					demandSum = demandSum
							+ instance.getNode(nodeIndex).getDemand()[toTimeIndex];
					IloNumExpr rouExpr = cplex.constant(0);
					for (int vehicleIndex = 0; vehicleIndex < instance
							.getNoNodes(); vehicleIndex++) {
						for (int otherNodeIndex = 0; otherNodeIndex < instance
								.getNoNodes(); otherNodeIndex++) {
							rouExpr = cplex
									.sum(rouExpr,
											routeVar[toTimeIndex][vehicleIndex][otherNodeIndex][nodeIndex]);
						}
					}
					expr = cplex.sum(expr, cplex.prod(-1, rouExpr));
					if (demandSum >= 0) {
						bw.write(cplex.addGe(invVar[timeIndex][nodeIndex],
								cplex.prod(demandSum, expr)) + "");
						bw.newLine();
					} else if (demandSum < 0) {
						bw.write(cplex.addGe(cplex.sum(
								instance.getNode(nodeIndex).getMaxCapacity(),
								cplex.prod(-1, invVar[timeIndex][nodeIndex])),
								cplex.prod(-demandSum, expr))
								+ "");
						bw.newLine();
					}

				}
			}
		}
	}

	private static void createEq15Cuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][] invVar)
			throws IloException, IOException {
		bw.write("Inventory Demand Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						expr = cplex
								.sum(expr,
										routeVar[timeIndex][vehicleIndex][otherNodeIndex][nodeIndex]);
					}
				}
				bw.write(cplex.addGe(invVar[timeIndex][nodeIndex], cplex.prod(
						instance.getNode(nodeIndex).getDemand()[timeIndex],
						cplex.sum(1, cplex.prod(-1, expr))))
						+ "");
			}
		}
	}

	private static void createDemandInvDiffCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar, IloNumVar[][] invVar)
			throws IloException, IOException {
		bw.write("Demand Inventory Difference Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr allExpr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					IloNumExpr rouExpr = cplex.constant(0);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						rouExpr = cplex
								.sum(rouExpr,
										routeVar[timeIndex][vehicleIndex][otherNodeIndex][nodeIndex]);
					}
					rouExpr = cplex.prod(instance.getVehicle(vehicleIndex)
							.getVehicleCapacity(), rouExpr);
					allExpr = cplex.sum(allExpr, rouExpr);
				}
				bw.write(cplex.addLe(cplex.sum(instance.getNode(nodeIndex)
						.getDemand()[timeIndex], cplex.prod(-1,
						invVar[timeIndex][nodeIndex])), allExpr)
						+ "");
				bw.newLine();
			}
		}
	}

	private static void createStartInvDemDiffCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Demand Starting Inventory Difference Cuts:");
		bw.newLine();

		for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
			if (instance.getNode(nodeIndex).getDemand()[0] > instance.getNode(
					nodeIndex).getStartInv()) {
				IloNumExpr expr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						expr = cplex
								.sum(expr,
										routeVar[0][vehicleIndex][otherNodeIndex][nodeIndex]);
					}
				}
				bw.write(cplex.addGe(expr, 1) + "");
				bw.newLine();
			}
		}
	}

	private static void createMinNumberVisitCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Minumum Visit Cuts:");
		bw.newLine();
		for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
			for (int fromTimeIndex = 0; fromTimeIndex < instance.getTime(); fromTimeIndex++) {
				for (int toTimeIndex = 1; toTimeIndex < instance.getTime(); toTimeIndex++) {
					IloNumExpr expr = cplex.constant(0);
					for (int vehicleIndex = 0; vehicleIndex < instance
							.getNoNodes(); vehicleIndex++) {
						for (int timeDiffIndex = fromTimeIndex; timeDiffIndex < toTimeIndex; timeDiffIndex++) {
							for (int fromNodeIndex = 0; fromNodeIndex < instance
									.getNoNodes(); fromNodeIndex++) {
								expr = cplex
										.sum(expr,
												routeVar[timeDiffIndex][vehicleIndex][fromNodeIndex][nodeIndex]);
							}
						}
					}
					bw.write(cplex.addGe(
							expr,
							getMinimumVisits(instance, fromTimeIndex,
									toTimeIndex, nodeIndex))
							+ "");
					bw.newLine();
				}
			}
		}
	}

	private static int getMinimumVisits(Instance instance, int fromTimeIndex,
			int toTimeIndex, int nodeIndex) {
		int visits = 0;
		double sumDemand = 0;
		for (int time = fromTimeIndex; time < toTimeIndex; time++) {
			sumDemand = sumDemand
					+ instance.getNode(nodeIndex).getDemand()[time];
		}
		visits = (int) Math
				.min(Math.ceil((double) (instance.getNode(nodeIndex)
						.getMinCapacity() + sumDemand - instance.getNode(
						nodeIndex).getMaxCapacity())
						/ (double) instance.getVehicle(0).getVehicleCapacity()),
						Math.ceil((double) (instance.getNode(nodeIndex)
								.getMaxCapacity() + sumDemand - instance
								.getNode(nodeIndex).getMaxCapacity())
								/ (double) instance.getVehicle(0)
										.getVehicleCapacity()));
		return visits;
	}

	private static void createVehicleReturnConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Vehicle Return Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr visitExpr = cplex.constant(0);
				IloNumExpr routeExpr = cplex.constant(0);

				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						visitExpr = cplex
								.sum(visitExpr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
						routeExpr = cplex
								.sum(routeExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
					}
				}
				bw.write(cplex.addLe(routeExpr, cplex.sum(visitExpr, 1)) + "");
				bw.newLine();
			}
		}
	}

	private static void createvisitRouteOutLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables Out Link Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						IloNumExpr expr = cplex.constant(0);
						for (int nodeIndex = 0; nodeIndex < instance
								.getNoNodes(); nodeIndex++) {
							expr = cplex
									.sum(expr,
											routeVar[timeIndex][vehicleIndex][nodeIndex][toNodeIndex]);
						}
						bw.write(cplex
								.addGe(expr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createvisitRouteInLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables In Link Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						IloNumExpr expr = cplex.constant(0);
						for (int nodeIndex = 0; nodeIndex < instance
								.getNoNodes(); nodeIndex++) {
							expr = cplex
									.sum(expr,
											routeVar[timeIndex][vehicleIndex][fromNodeIndex][nodeIndex]);
						}
						bw.write(cplex
								.addGe(expr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createvisitQtyLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][] qtyVar,
			IloNumVar[][][][] visitVar) throws IloException, IOException {
		bw.write("Visit, Quantity Variables Link Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					IloNumExpr expr = cplex.constant(0);
					for (int vehicleIndex = 0; vehicleIndex < instance
							.getNoNodes(); vehicleIndex++) {
						expr = cplex
								.sum(expr,
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]));
					}
					bw.write(cplex
							.addLe(qtyVar[timeIndex][fromNodeIndex][toNodeIndex],
									expr)
							+ "");
					bw.newLine();
				}
			}
		}
	}

	private static void createInvBalance(Instance instance, IloCplex cplex,
			IloNumVar[][][] qtyVar, IloNumVar[][] invVar) throws IloException,
			IOException {
		bw.write("Inventory Balance Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr inExpr = cplex.constant(0);
				IloNumExpr outExpr = cplex.constant(0);
				for (int otherNodeIndex = 0; otherNodeIndex < instance
						.getNoNodes(); otherNodeIndex++) {
					inExpr = cplex.sum(inExpr,
							qtyVar[timeIndex][otherNodeIndex][nodeIndex]);
					outExpr = cplex.sum(outExpr,
							qtyVar[timeIndex][nodeIndex][otherNodeIndex]);
				}

				bw.write(cplex.addEq(invVar[timeIndex + 1][nodeIndex], cplex
						.sum(invVar[timeIndex][nodeIndex], inExpr, cplex.prod(
								-1, outExpr), cplex.constant(-instance.getNode(
								nodeIndex).getDemand()[timeIndex])))
						+ "");
				bw.newLine();
			}
		}
	}

	private static void createMaxDistanceConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Maximum Distance Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 1; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						expr = cplex
								.sum(expr,
										cplex.prod(
												routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
												instance.distanceMatrix[fromNodeIndex][toNodeIndex]));
					}
				}
				bw.write(cplex.addLe(expr, instance.getVehicle(vehicleIndex)
						.getMaxDistance())
						+ "");
				bw.newLine();
			}
		}
	}

	private static void setParameters(IloCplex cplex) throws IloException {
		cplex.setParam(IloCplex.DoubleParam.TiLim, 18000);
		cplex.setOut(logwriter);

	}

	private static void createVehicleStartNodeConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Vehicle Start Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr routExpr = cplex.constant(0);
				IloNumExpr focusNodeExpr = cplex.constant(0);
				for (int focusNodeIndex = 0; focusNodeIndex < instance
						.getNoNodes(); focusNodeIndex++) {
					focusNodeExpr = cplex
							.sum(focusNodeExpr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][focusNodeIndex]);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						routExpr = cplex
								.sum(routExpr,
										routeVar[timeIndex][vehicleIndex][focusNodeIndex][otherNodeIndex]);
					}
				}
				bw.write(cplex.addLe(routExpr,
						cplex.prod(instance.getNoNodes(), focusNodeExpr))
						+ "\n");
			}
		}
	}

	private static void createRouteBalConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Route Balance Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					IloNumExpr inExpr = cplex.constant(0);
					IloNumExpr outExpr = cplex.constant(0);
					for (int otherIndex = 0; otherIndex < instance.getNoNodes(); otherIndex++) {
						inExpr = cplex
								.sum(inExpr,
										routeVar[timeIndex][vehicleIndex][otherIndex][nodeIndex]);
						outExpr = cplex
								.sum(outExpr,
										routeVar[timeIndex][vehicleIndex][nodeIndex][otherIndex]);
					}
					bw.write(cplex.addEq(inExpr, outExpr) + "");
					bw.newLine();
				}
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

	private static IloNumVar[][][][] createRouteVariablesIP(IloCplex cplex,
			Instance instance, String name) throws IloException, IOException {
		IloNumVar[][][][] rouVar = new IloNumVar[instance.getTime()][][][];
		for (int timeIndex = 0; timeIndex < rouVar.length; timeIndex++) {
			rouVar[timeIndex] = new IloNumVar[instance.getNoNodes()][][];
			for (int vehicleIndex = 0; vehicleIndex < rouVar[timeIndex].length; vehicleIndex++) {
				rouVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()][];
				for (int fromIndex = 0; fromIndex < rouVar[timeIndex][vehicleIndex].length; fromIndex++) {
					rouVar[timeIndex][vehicleIndex][fromIndex] = new IloNumVar[instance
							.getNoNodes()];
					for (int toIndex = 0; toIndex < rouVar[timeIndex][vehicleIndex][fromIndex].length; toIndex++) {
						rouVar[timeIndex][vehicleIndex][fromIndex][toIndex] = cplex
								.numVar(0, 1, IloNumVarType.Bool, name + "^{"
										+ timeIndex + "," + vehicleIndex + "}"
										+ "_{" + fromIndex + "," + toIndex
										+ "}");
					}
				}
			}
		}
		return rouVar;
	}

	private static IloNumVar[][][][] createRouteVariablesLP(IloCplex cplex,
			Instance instance, String name) throws IloException, IOException {
		IloNumVar[][][][] rouVar = new IloNumVar[instance.getTime()][][][];
		for (int timeIndex = 0; timeIndex < rouVar.length; timeIndex++) {
			rouVar[timeIndex] = new IloNumVar[instance.getNoNodes()][][];
			for (int vehicleIndex = 0; vehicleIndex < rouVar[timeIndex].length; vehicleIndex++) {
				rouVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()][];
				for (int fromIndex = 0; fromIndex < rouVar[timeIndex][vehicleIndex].length; fromIndex++) {
					rouVar[timeIndex][vehicleIndex][fromIndex] = new IloNumVar[instance
							.getNoNodes()];
					for (int toIndex = 0; toIndex < rouVar[timeIndex][vehicleIndex][fromIndex].length; toIndex++) {
						rouVar[timeIndex][vehicleIndex][fromIndex][toIndex] = cplex
								.numVar(0, 1, IloNumVarType.Float, name + "^{"
										+ timeIndex + "," + vehicleIndex + "}"
										+ "_{" + fromIndex + "," + toIndex
										+ "}");
					}
				}
			}
		}
		return rouVar;
	}

	private static IloNumVar[][][] createQuantityVariables(IloCplex cplex,
			Instance instance) throws IloException {
		IloNumVar[][][] qtyVar = new IloNumVar[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < qtyVar.length; timeIndex++) {
			qtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];

			for (int fromIndex = 0; fromIndex < qtyVar[timeIndex].length; fromIndex++) {
				qtyVar[timeIndex][fromIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int toIndex = 0; toIndex < qtyVar[timeIndex][fromIndex].length; toIndex++) {

					qtyVar[timeIndex][fromIndex][toIndex] = cplex.numVar(0,
							Double.MAX_VALUE, IloNumVarType.Float, "q^"
									+ timeIndex + "_{" + fromIndex + ","
									+ toIndex + "}");
				}
			}
		}
		return qtyVar;
	}

	private static IloNumVar[][][] createNextVaribles(IloCplex cplex,
			Instance instance) throws IloException, IOException {

		IloNumVar[][][] visitVar = new IloNumVar[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < visitVar.length; timeIndex++) {
			visitVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];

			for (int vehicleIndex = 0; vehicleIndex < visitVar[timeIndex].length; vehicleIndex++) {
				visitVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int nodeIndex = 0; nodeIndex < visitVar[timeIndex][vehicleIndex].length; nodeIndex++) {

					visitVar[timeIndex][vehicleIndex][nodeIndex] = cplex
							.numVar(0, Double.MAX_VALUE, IloNumVarType.Float,
									"u^{" + timeIndex + "," + vehicleIndex
											+ "}_" + nodeIndex);
				}
			}
		}
		return visitVar;
	}

	private static String getTargetLoc(String fileName) {
		String lastfolder = fileName.substring(0, fileName.lastIndexOf("."));
		lastfolder = lastfolder.replace("TestCases", "TestCases\\Visit\\");
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

	private static void getSolution(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar, IloNumVar[][][] qtyVar,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar,
			IloNumVar[][][] nextVar) {
		try {
			solwriter.write("Objective Value: " + cplex.getObjValue() + "\n");
			solwriter.write("Inventory Variables:\n");
			for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					solwriter.write(invVar[timeIndex][nodeIndex].getName()
							+ ":"
							+ cplex.getValue(invVar[timeIndex][nodeIndex]));
					solwriter.write("\t");
					solwriter.flush();
				}
				solwriter.newLine();
			}

			solwriter.write("Quantity Variables:\n");
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
											+ "; ");
						}

					}
					solwriter.newLine();
				}
				solwriter.write("************************");
				solwriter.newLine();
			}

			solwriter.write("Route Variables:\n");
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {
							if (cplex
									.getValue(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0) {
								solwriter
										.write(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ "; ");
							}
						}
						solwriter.write("\t");
					}
					solwriter.newLine();
				}
				solwriter.write("************************");
				solwriter.newLine();
			}

			solwriter.write("Visit Variables:\n");
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {
							if (cplex
									.getValue(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0) {
								solwriter
										.write(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ "; ");
							}
						}
						solwriter.write("\t");
					}
					solwriter.newLine();
				}
				solwriter.write("################");
				solwriter.newLine();
			}

			// solwriter.write("Next Variables:\n");
			// for (int timeIndex = 0; timeIndex < instance.getTime();
			// timeIndex++) {
			// for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes();
			// vehicleIndex++) {
			//
			// for (int nodeIndex = 0; nodeIndex < instance.getNoNodes();
			// nodeIndex++) {
			// if (cplex
			// .getValue(nextVar[timeIndex][vehicleIndex][nodeIndex]) != 0) {
			// solwriter
			// .write(nextVar[timeIndex][vehicleIndex][nodeIndex]
			// .getName()
			// + ":"
			// + cplex.getValue(nextVar[timeIndex][vehicleIndex][nodeIndex])
			// + ";");
			// }
			//
			// }
			// solwriter.newLine();
			// }
			// solwriter.write("_________________");
			// solwriter.newLine();
			// }

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}