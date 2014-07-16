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
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class solveMIRPTForm6 {

	public static BufferedWriter bw = null;
	public static BufferedWriter solwriter = null;
	public static OutputStream logwriter = null;
	public static BufferedWriter mapwriter = null;

	public static void main(String[] args) {
		String inputfile = "D:\\The Folder\\IRP\\TestCases\\Problems";
		solveProblems(inputfile);
		System.out
				.println("________________Formulation 6 Done!!!________________");
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
											+ "-F6-"
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
											+ "-F6-"
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
									+ "-F6-"
									+ slp
									+ scuts
									+ (new SimpleDateFormat("MMM-dd--HH-mm-ss"))
											.format(new Date()) + "-Log.txt");

					mapwriter = new BufferedWriter(
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ fileName.substring(
													fileName.lastIndexOf("\\") + 1,
													fileName.indexOf("."))
											+ "-F6-"
											+ slp
											+ scuts
											+ (new SimpleDateFormat(
													"MMM-dd--HH-mm-ss"))
													.format(new Date())
											+ "-Map.tex"));
				} catch (IOException e) {
					e.printStackTrace();
				}

				processProblem(instance, lp, cuts, fileName);

				try {
					solwriter.close();
					bw.close();
					mapwriter.write("\\end{document}");
					mapwriter.close();
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
			IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
			IloNumVar[][][] inQtyVar = createQuantityVariables(cplex, instance,
					"iq");
			IloNumVar[][][] outQtyVar = createQuantityVariables(cplex,
					instance, "oq");
			IloNumVar[][][][] qtyTranVar = createQuantityTransferVariables(
					cplex, instance);

			IloNumVar[][][][] routeVar = null;
			IloNumVar[][][][] visitVar = null;
			if (lp) {
				routeVar = createRouteVariablesLP(cplex, instance, "y");
				visitVar = createRouteVariablesLP(cplex, instance, "z");
			} else {
				routeVar = createRouteVariablesIP(cplex, instance, "y");
				visitVar = createRouteVariablesIP(cplex, instance, "z");
			}

			createConstraints(cplex, instance, cuts, invVar, inQtyVar,
					outQtyVar, qtyTranVar, routeVar, visitVar);

			createObjective(cplex, instance, routeVar, invVar);

			setParameters(cplex);
			long startTime = System.nanoTime();
			// cplex.exportModel("E:\\MIRPTForm6.mps");
			if (cplex.solve()) {
				System.out.println(fileName.substring(fileName
						.lastIndexOf("\\") + 1)
						+ "-Solved-"
						+ cplex.getObjValue()
						+ "- in Time-"
						+ ((System.nanoTime() - startTime) / 1E9) + " sec");
				getSolution(cplex, instance, invVar, inQtyVar, outQtyVar,
						routeVar, visitVar, qtyTranVar);
			} else {
				System.out.println("Couldn't Solve!!" + cplex.getStatus());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void createConstraints(IloCplex cplex, Instance instance,
			boolean cuts, IloNumVar[][] invVar, IloNumVar[][][] inQtyVar,
			IloNumVar[][][] outQtyVar, IloNumVar[][][][] qtyTranVar,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar) {
		try {
			createInvBalance(instance, cplex, invVar, inQtyVar, outQtyVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createQuanTranLinkConstraints(cplex, instance, inQtyVar, outQtyVar,
					qtyTranVar);
			createTransferVisitLinkConstraints(cplex, instance, qtyTranVar,
					visitVar);

			createvisitRouteInLinkConstraints(cplex, instance, visitVar,
					routeVar);
			createvisitRouteOutLinkConstraints(cplex, instance, visitVar,
					routeVar);

			createRouteBalConstraints(cplex, instance, routeVar);
			createOneVehiclePerNodeConstraints(cplex, instance, routeVar);
			createVehicleStartNodeConstraints(cplex, instance, routeVar);
			createMaxDistanceConstraints(cplex, instance, routeVar);

			if (cuts) {
				// Min Visit Cuts
				createMinNumberVisitCuts(cplex, instance, routeVar);

				//
				createDemandInvDiffCuts(cplex, instance, routeVar, invVar);

				// IDD cuts
				createStartInvDemDiffCuts(cplex, instance, routeVar);

				// eq14 Cuts
				createEq14Cuts(cplex, instance, routeVar, invVar);

				createvisitRouteInLinkCuts(cplex, instance, visitVar, routeVar);
				createvisitRouteOutLinkCuts(cplex, instance, visitVar, routeVar);

				// // z>=y cuts
				// createVisitRouteCuts(cplex, instance, routeVar, visitVar);
				//
				// // sigma z^{ti}_{ij} + 1 >= sigma sigma y^{ti}_{jk} cuts
				// createSumZGYCuts(cplex, instance, routeVar, visitVar);
				//
				// // sigma sigma y^{ti}_{jk} >= sigma z^{ti}_{ij} cuts
				// createSumYGSumZCuts(cplex, instance, routeVar, visitVar);

				// // z_{ij}+z_{ji}<=sigma y^i_{ik}
				// createZijZjiCuts(cplex, instance, visitVar, routeVar);

				// // y_{ij}+y_{ji}<=sigma y^i_{ik}
				// createYijYjiCuts(cplex, instance, routeVar);

			}

		} catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static void createYijYjiCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("y_{ij}+y_{ji}<=sigma y^i_{ik} cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {

				IloNumExpr yExpr = cplex.constant(0);
				for (int otherNodeIndex = 0; otherNodeIndex < instance
						.getNoNodes(); otherNodeIndex++) {
					if (otherNodeIndex == vehicleIndex)
						continue;
					yExpr = cplex
							.sum(yExpr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][otherNodeIndex]);
				}
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					if (fromNodeIndex == vehicleIndex)
						continue;
					for (int toNodeIndex = fromNodeIndex + 1; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (toNodeIndex == vehicleIndex)
							continue;
						bw.write(cplex.addLe(
								cplex.sum(
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
										routeVar[timeIndex][vehicleIndex][toNodeIndex][fromNodeIndex]),
								yExpr)
								+ "\n");
					}
				}

			}

		}

	}

	private static void createvisitRouteOutLinkCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables Out Link Cuts(Individual Nodes):\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex)
							continue;
						IloNumExpr expr = cplex.constant(0);
						for (int otherNodeIndex = 0; otherNodeIndex < instance
								.getNoNodes(); otherNodeIndex++) {
							if (otherNodeIndex == toNodeIndex)
								continue;
							expr = cplex
									.sum(expr,
											routeVar[timeIndex][vehicleIndex][otherNodeIndex][toNodeIndex]);
						}
						bw.write(cplex.addLazyConstraint(cplex
								.ge(expr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createvisitRouteInLinkCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables In Link Cuts(Individual Nodes):\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex)
							continue;
						IloNumExpr expr = cplex.constant(0);
						for (int otherNodeIndex = 0; otherNodeIndex < instance
								.getNoNodes(); otherNodeIndex++) {
							if (otherNodeIndex == fromNodeIndex)
								continue;
							expr = cplex
									.sum(expr,
											routeVar[timeIndex][vehicleIndex][fromNodeIndex][otherNodeIndex]);
						}
						bw.write(cplex.addLazyConstraint(cplex
								.ge(expr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createZijZjiCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] visitVar, IloNumVar[][][][] routeVar)
			throws IloException, IOException {
		bw.write("z_{ij}+z_{ji}<=sigma y^i_{ik} cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {

				IloNumExpr yExpr = cplex.constant(0);
				for (int otherNodeIndex = 0; otherNodeIndex < instance
						.getNoNodes(); otherNodeIndex++) {
					if (otherNodeIndex == vehicleIndex)
						continue;
					yExpr = cplex
							.sum(yExpr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][otherNodeIndex]);
				}
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = fromNodeIndex + 1; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {

						bw.write(cplex.addLe(
								cplex.sum(
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
										visitVar[timeIndex][vehicleIndex][toNodeIndex][fromNodeIndex]),
								yExpr)
								+ "\n");
					}
				}

			}

		}

	}

	private static void createSumYGSumZCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar)
			throws IloException, IOException {
		bw.write("sigma sigma y^{ti}_{jk} = sigma z^{ti}_{ij} cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr zExpr = cplex.constant(0);
				IloNumExpr yExpr = cplex.constant(0);

				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					if (vehicleIndex == fromNodeIndex) {
						continue;
					}
					zExpr = cplex
							.sum(zExpr,
									visitVar[timeIndex][vehicleIndex][vehicleIndex][fromNodeIndex]);
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex) {
							continue;
						}
						yExpr = cplex
								.sum(yExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
					}
				}
				bw.write(cplex.addEq(yExpr, zExpr) + "\n");
			}
		}
	}

	private static void createSumZGYCuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][][][] visitVar)
			throws IloException, IOException {
		bw.write("Sigma Z + 1 >= Sigma Y ;Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr zExpr = cplex.constant(0);
				IloNumExpr yExpr = cplex.constant(0);

				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					if (fromNodeIndex == vehicleIndex) {
						continue;
					}
					zExpr = cplex
							.sum(zExpr,
									visitVar[timeIndex][vehicleIndex][vehicleIndex][fromNodeIndex]);
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex) {
							continue;
						}
						yExpr = cplex
								.sum(yExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
					}
				}
				bw.write(cplex.addGe(cplex.sum(zExpr, 1), yExpr) + "\n");
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
							if (otherNodeIndex == nodeIndex)
								continue;
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

	private static void createStartInvDemDiffCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Demand Starting Inventory Difference Cuts:");
		bw.newLine();

		for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
			if (instance.getNode(nodeIndex).getDemand()[0] > instance.getNode(
					nodeIndex).getStartInv()) {
				IloNumExpr expr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						if (otherNodeIndex == nodeIndex)
							continue;
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

	private static void createDemandInvDiffCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar, IloNumVar[][] invVar)
			throws IloException, IOException {
		bw.write("Demand Inventory Difference Cuts:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr allExpr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					IloNumExpr rouExpr = cplex.constant(0);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						if (otherNodeIndex == nodeIndex) {
							continue;
						}
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
								if (fromNodeIndex == nodeIndex) {
									continue;
								}
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

		visits = (int) Math.min(Math.ceil(((double) (instance
				.getNode(nodeIndex).getMinCapacity() + sumDemand - instance
				.getNode(nodeIndex).getMaxCapacity()) / (double) instance
				.getVehicle(0).getVehicleCapacity())), Math
				.ceil(((double) (sumDemand) / (double) instance.getVehicle(0)
						.getVehicleCapacity())));
		// System.out.println(((double) (instance.getNode(nodeIndex)
		// .getMinCapacity() + sumDemand - instance.getNode(nodeIndex)
		// .getMaxCapacity()) / (double) instance.getVehicle(0)
		// .getVehicleCapacity())
		// + ":"
		// + ((double) (sumDemand) / (double) instance.getVehicle(0)
		// .getVehicleCapacity()) + ";v:" + visits);
		return visits;
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
						if (fromNodeIndex == toNodeIndex)
							continue;
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

	private static void createvisitRouteOutLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables Out Link Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {

				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {

					IloNumExpr rouExpr = cplex.constant(0);
					IloNumExpr visitExpr = cplex.constant(0);
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						if (fromNodeIndex == toNodeIndex)
							continue;
						rouExpr = cplex
								.sum(rouExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);
						visitExpr = cplex
								.sum(visitExpr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);

					}
					bw.write(cplex.addGe(
							cplex.prod(rouExpr, instance.getNoNodes() - 1),
							visitExpr) + "");
					bw.newLine();
				}
			}
		}
	}

	private static void createvisitRouteInLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] visitVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route, Visit Variables In Link Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					IloNumExpr rouExpr = cplex.constant(0);
					IloNumExpr visitExpr = cplex.constant(0);
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex)
							continue;

						visitExpr = cplex
								.sum(visitExpr,
										visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);

						rouExpr = cplex
								.sum(rouExpr,
										routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]);

					}
					bw.write(cplex.addGe(
							cplex.prod(rouExpr, instance.getNoNodes() - 1),
							visitExpr) + "");
					bw.newLine();
				}
			}
		}
	}

	private static void createMaxDistanceConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Maximum Distance Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 1; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex)
							continue;
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

	private static void createVehicleStartNodeConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Vehicle Start Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr routExpr = cplex.constant(0);
				IloNumExpr focusNodeExpr = cplex.constant(0);
				for (int focusNodeIndex = 0; focusNodeIndex < instance
						.getNoNodes(); focusNodeIndex++) {
					if (vehicleIndex == focusNodeIndex)
						continue;
					focusNodeExpr = cplex
							.sum(focusNodeExpr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][focusNodeIndex]);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						if (focusNodeIndex == otherNodeIndex)
							continue;
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

	private static void createOneVehiclePerNodeConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Max One Vehicle per Node Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					if (vehicleIndex == nodeIndex)
						continue;
					expr = cplex
							.sum(expr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][nodeIndex]);
				}
				bw.write(cplex.addLe(expr, 1) + "\n");
			}
		}
	}

	private static void createTransferVisitLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] qtyTranVar,
			IloNumVar[][][][] visitVar) throws IloException, IOException {
		bw.write("Transfer Route Link Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						if (fromNodeIndex == toNodeIndex) {
							continue;
						}

						bw.write(cplex
								.addLe(qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex],
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))
								+ "\n");
					}
				}
			}
		}
	}

	private static void createQuanTranLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][] inQtyVar,
			IloNumVar[][][] outQtyVar, IloNumVar[][][][] qtyTranVar)
			throws IloException, IOException {
		bw.write("Quantity Transfer Link Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					IloNumExpr inExpr = cplex.constant(0);
					IloNumExpr outExpr = cplex.constant(0);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						if (nodeIndex == otherNodeIndex) {
							continue;
						}
						inExpr = cplex
								.sum(inExpr,
										qtyTranVar[timeIndex][vehicleIndex][otherNodeIndex][nodeIndex]);
						outExpr = cplex
								.sum(outExpr,
										qtyTranVar[timeIndex][vehicleIndex][nodeIndex][otherNodeIndex]);
					}
					bw.write(cplex.addEq(inExpr,
							inQtyVar[timeIndex][vehicleIndex][nodeIndex])
							+ "\n");
					bw.write(cplex.addEq(outExpr,
							outQtyVar[timeIndex][vehicleIndex][nodeIndex])
							+ "\n");
				}
			}
		}
	}

	private static void createRouteBalConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Route Balance Constraints:\n");

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
		bw.write("Maximum Inventory Constraints:\n");

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
		bw.write("Minimum Inventory Constraints:\n");

		for (int timeIndex = 1; timeIndex < instance.getTime() + 1; timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				bw.write(cplex.addLe(instance.getNode(nodeIndex)
						.getMinCapacity(), invVar[timeIndex][nodeIndex])
						+ "");
				bw.newLine();
			}
		}
	}

	private static void createInvBalance(Instance instance, IloCplex cplex,
			IloNumVar[][] invVar, IloNumVar[][][] inQtyVar,
			IloNumVar[][][] outQtyVar) throws IloException, IOException {
		bw.write("Inventory Balance Constraints:\n");

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr inExpr = cplex.constant(0);
				IloNumExpr outExpr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					inExpr = cplex.sum(inExpr,
							inQtyVar[timeIndex][vehicleIndex][nodeIndex]);
					outExpr = cplex.sum(outExpr,
							outQtyVar[timeIndex][vehicleIndex][nodeIndex]);
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

	private static void setParameters(IloCplex cplex) throws IloException {
		cplex.setParam(IloCplex.DoubleParam.TiLim, 18000);
		cplex.setOut(logwriter);
	}

	private static IloNumVar[][][][] createQuantityTransferVariables(
			IloCplex cplex, Instance instance) throws IloException, IOException {
		IloNumVar[][][][] qtyTranVar = new IloNumVar[instance.getTime()][][][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			qtyTranVar[timeIndex] = new IloNumVar[instance.getNoNodes()][][];
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				qtyTranVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()][];
				for (int fromNodeIndex = 0; fromNodeIndex < instance
						.getNoNodes(); fromNodeIndex++) {
					qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex] = new IloNumVar[instance
							.getNoNodes()];
					for (int toNodeIndex = 0; toNodeIndex < instance
							.getNoNodes(); toNodeIndex++) {
						qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex] = cplex
								.numVar(0, Double.MAX_VALUE,
										IloNumVarType.Float, "x^{" + timeIndex
												+ "," + vehicleIndex + "}_{"
												+ fromNodeIndex + ","
												+ toNodeIndex + "}");
					}
				}
			}
		}
		return qtyTranVar;
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

	private static IloNumVar[][][] createQuantityVariables(IloCplex cplex,
			Instance instance, String dir) throws IloException, IOException {
		IloNumVar qtyVar[][][] = new IloNumVar[instance.getTime() + 1][][];
		for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
			qtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				qtyVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					qtyVar[timeIndex][vehicleIndex][nodeIndex] = cplex.numVar(
							0, Double.MAX_VALUE, IloNumVarType.Float, dir
									+ "^{" + timeIndex + "," + vehicleIndex
									+ "}_" + nodeIndex);

				}
			}
		}
		return qtyVar;
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
						// if (fromIndex == toIndex) {
						// rouVar[timeIndex][vehicleIndex][fromIndex][toIndex] =
						// cplex
						// .numVar(0, 0, IloNumVarType.Float, name
						// + "^{" + timeIndex + ","
						// + vehicleIndex + "}" + "_{"
						// + fromIndex + "," + toIndex + "}");
						// } else
						{
							rouVar[timeIndex][vehicleIndex][fromIndex][toIndex] = cplex
									.numVar(0, 1, IloNumVarType.Float, name
											+ "^{" + timeIndex + ","
											+ vehicleIndex + "}" + "_{"
											+ fromIndex + "," + toIndex + "}");
						}
					}
				}
			}
		}
		return rouVar;
	}

	private static String getTargetLoc(String fileName) {
		String lastfolder = fileName.substring(0, fileName.lastIndexOf("."));
		lastfolder = lastfolder.replace("TestCases",
				"TestCases\\Form6QtyPathVisit\\");
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

	private static void getSolution(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar, IloNumVar[][][] inQtyVar,
			IloNumVar[][][] outQtyVar, IloNumVar[][][][] routeVar,
			IloNumVar[][][][] visitVar, IloNumVar[][][][] qtyTranVar) {
		try {

			solwriter.write("Objective Value: " + cplex.getObjValue() + "\n");
			mapwriter.write("\\documentclass[a4paper]{article}\n"
					+ "\\usepackage{tikz,float} \n"
					+ "\\usetikzlibrary{shapes.geometric,positioning} \n"
					+ "\\begin{document} \n" + "Objective Value: "
					+ cplex.getObjValue() + "\n\n");

			solwriter.write("Inventory Variables:\n");

			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				mapwriter.write("Time Period:" + timeIndex + "\n\n");
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					solwriter.write(invVar[timeIndex][nodeIndex].getName()
							+ ":"
							+ cplex.getValue(invVar[timeIndex][nodeIndex]));
					solwriter.write("\t");
					solwriter.flush();
				}
				solwriter.newLine();
				solwriter.write("In Quantity for period:" + timeIndex + "\n");

				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {

						if (cplex
								.getValue(inQtyVar[timeIndex][vehicleIndex][nodeIndex]) != 0) {
							solwriter
									.write(inQtyVar[timeIndex][vehicleIndex][nodeIndex]
											.getName()
											+ ":"
											+ cplex.getValue(inQtyVar[timeIndex][vehicleIndex][nodeIndex])
											+ "; ");
						}

					}
					solwriter.newLine();
				}
				solwriter.write("------------\n");

				solwriter.write("Out Quantity for period:" + timeIndex + "\n");

				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
						if (cplex
								.getValue(outQtyVar[timeIndex][vehicleIndex][nodeIndex]) != 0) {
							solwriter
									.write(outQtyVar[timeIndex][vehicleIndex][nodeIndex]
											.getName()
											+ ":"
											+ cplex.getValue(outQtyVar[timeIndex][vehicleIndex][nodeIndex])
											+ "; ");
						}

					}
					solwriter.newLine();
				}
				solwriter.write("------------\n************************");
				solwriter.newLine();

				solwriter.write("Quantity Variables:\n");
				mapwriter
						.write("Quantity Transfer Variables($x_{ij}^{tv}$):\n\n");

				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					boolean open = true;

					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {

							if (getCplexValue(
									cplex,
									qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0) {
								solwriter
										.write(qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ "; ");
								if (open) {
									mapwriter
											.write("\\begin{figure}[H] \n"
													+ "\\centering \n"
													+ "\\begin{tikzpicture} \n"
													+ "\\node (n0) [ellipse, draw=black, text=black, scale=1]{$0$}; \n"
													+ "\\node (n1) [ellipse, draw=black, text=black, scale=1, below left=6cm of n0]{$1$}; \n"
													+ "\\node (n4) [ellipse, draw=black, text=black, scale=1, below right =6cm of n0]{$4$}; \n"
													+ "\\node (n2) [ellipse, draw=black,text=black,scale=1, below right =3cm of n1]{$2$}; \n"
													+ "\\node (n3) [ellipse, draw=black, text=black, scale=1, below left =3cm of n4]{$3$};\n");
									open = false;
								}
								mapwriter
										.write(getArcforMap(
												fromNodeIndex,
												toNodeIndex,
												getformattedNumber(cplex
														.getValue(qtyTranVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))));
							}
						}
						solwriter.write("\t");
					}
					if (!open) {
						mapwriter.write("\\end{tikzpicture}\n"
								+ "\\caption{$x^{" + timeIndex + ""
								+ vehicleIndex + "}_{ij}$ Time:" + timeIndex
								+ " Vehicle:" + vehicleIndex + "} \n"
								+ "\\label{qt" + timeIndex + "v" + vehicleIndex
								+ "}\n" + "\\end{figure}\n\n");
					}
					solwriter.newLine();
				}
				solwriter.write("******************************************");
				solwriter.newLine();

				solwriter.write("Visit Variables:\n");
				mapwriter.write("Visit Variables($z_{ij}^{tv}$):\n\n");
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {

					boolean open = true;
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {
							if (getCplexValue(
									cplex,
									visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0) {
								solwriter
										.write(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ "; ");
								if (open) {
									mapwriter
											.write("\\begin{figure}[H] \n"
													+ "\\centering \n"
													+ "\\begin{tikzpicture} \n"
													+ "\\node (n0) [ellipse, draw=black, text=black, scale=1]{$0$}; \n"
													+ "\\node (n1) [ellipse, draw=black, text=black, scale=1, below left=6cm of n0]{$1$}; \n"
													+ "\\node (n4) [ellipse, draw=black, text=black, scale=1, below right =6cm of n0]{$4$}; \n"
													+ "\\node (n2) [ellipse, draw=black,text=black,scale=1, below right =3cm of n1]{$2$}; \n"
													+ "\\node (n3) [ellipse, draw=black, text=black, scale=1, below left =3cm of n4]{$3$};\n");
									open = false;
								}
								mapwriter
										.write(getArcforMap(
												fromNodeIndex,
												toNodeIndex,
												get3digitNumber(cplex
														.getValue(visitVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))));

							}
						}
						solwriter.write("\t");
					}
					if (!open) {
						mapwriter.write("\\end{tikzpicture}\n"
								+ "\\caption{$z^{" + timeIndex + ""
								+ vehicleIndex + "}_{ij}$ Time:" + timeIndex
								+ " Vehicle:" + vehicleIndex + "} \n"
								+ "\\label{zt" + timeIndex + "v" + vehicleIndex
								+ "}\n" + "\\end{figure}\n\n");
					}
					solwriter.newLine();
				}
				solwriter.write("_______________________________");
				solwriter.newLine();

				solwriter.write("Route Variables:\n");
				mapwriter.write("Route Variables($y_{ij}^{tv}$):\n\n");
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {

					boolean open = true;
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
								if (open) {
									mapwriter
											.write("\\begin{figure}[H] \n"
													+ "\\centering \n"
													+ "\\begin{tikzpicture} \n"
													+ "\\node (n0) [ellipse, draw=black, text=black, scale=1]{$0$}; \n"
													+ "\\node (n1) [ellipse, draw=black, text=black, scale=1, below left=6cm of n0]{$1$}; \n"
													+ "\\node (n4) [ellipse, draw=black, text=black, scale=1, below right =6cm of n0]{$4$}; \n"
													+ "\\node (n2) [ellipse, draw=black,text=black,scale=1, below right =3cm of n1]{$2$}; \n"
													+ "\\node (n3) [ellipse, draw=black, text=black, scale=1, below left =3cm of n4]{$3$};\n");
									open = false;
								}
								mapwriter
										.write(getArcforMap(
												fromNodeIndex,
												toNodeIndex,
												get3digitNumber(cplex
														.getValue(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]))));
							}
						}
						solwriter.write("\t");
					}
					if (!open) {
						mapwriter.write("\\end{tikzpicture}\n"
								+ "\\caption{$y^{" + timeIndex + ""
								+ vehicleIndex + "}_{ij}$ Time:" + timeIndex
								+ " Vehicle:" + vehicleIndex + "} \n"
								+ "\\label{yt" + timeIndex + "v" + vehicleIndex
								+ "}\n" + "\\end{figure}\n\n");
					}
					solwriter.newLine();
				}
				solwriter.write("########################");
				solwriter.newLine();
			}
			solwriter.write("Inventory at the end of horizon:\n");
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				solwriter
						.write(invVar[instance.getTime()][nodeIndex].getName()
								+ ":"
								+ cplex.getValue(invVar[instance.getTime()][nodeIndex]));
				solwriter.write("\t");
				solwriter.flush();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static double getCplexValue(IloCplex cplex, IloNumVar iloNumVar) {
		double value;
		try {
			value = cplex.getValue(iloNumVar);
		} catch (IloException e) {
			return 0;
		}
		return value;
	}

	private static String getformattedNumber(double value) {
		DecimalFormat df = new DecimalFormat("0.##");
		return df.format(value);
	}

	private static String get3digitNumber(double value) {
		DecimalFormat df = new DecimalFormat("0.######");
		return df.format(value);
	}

	private static String getArcforMap(int fromNodeIndex, int toNodeIndex,
			String value) {
		String arc = null;
		String[][] arcSet = {
				{
						"\\draw[bend right, ->] (n0)..controls  +(up:2cm) and +(right:2cm)..(n0) node [pos=0.5,sloped, above]{value};\n",
						"\\draw[->]  (n0) to node [pos=0.15,sloped, above ] {\\begin{small}value\\end{small}} (n1);\n",
						"\\draw[->]  (n0) to node [pos=0.1,sloped, above] {\\begin{small}value\\end{small}} (n2);\n",
						"\\draw[->]  (n0) to node [pos=0.15,sloped, above] {\\begin{small}value\\end{small}} (n3);\n",
						"\\draw[->]  (n0) to node [pos=0.15,sloped, above] {\\begin{small}value\\end{small}} (n4);\n"},
				{
						"\\draw[bend left,->]  (n1) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n0);\n",
						"\\draw[bend right, ->] (n1)..controls +(left:2cm) and +(up:2cm)..(n1) node [pos=0.5,sloped, above]{value};\n",
						"\\draw[->]  (n1) to node [pos=0.3,sloped, above] {\\begin{small}value\\end{small}} (n2);\n",
						"\\draw[->]  (n1) to node [pos=0.15,sloped, above] {\\begin{small}value\\end{small}} (n3);\n",
						"\\draw[->]  (n1) to node [pos=0.1,sloped, above] {\\begin{small}value\\end{small}} (n4);\n"},
				{
						"\\draw[bend left,->]  (n2) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n0);\n",
						"\\draw[bend left,->]  (n2) to node [pos=.15, sloped, above] {\\begin{small}value\\end{small}} (n1);\n",
						"\\draw[bend right, ->] (n2)..controls +(left:2cm) and +(down:2cm)..(n2) node [pos=0.5,sloped, below]{value};\n",
						"\\draw[->]  (n2) to node [pos=0.3,sloped, above] {\\begin{small}value\\end{small}} (n3);\n",
						"\\draw[->]  (n2) to node [pos=0.15,sloped, above] {\\begin{small}value\\end{small}} (n4);\n"},
				{
						"\\draw[bend right,->]  (n3) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n0);\n",
						"\\draw[bend right,->]  (n3) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n1);\n",
						"\\draw[bend left,->]  (n3) to node [pos=.25, sloped, above] {\\begin{small}value\\end{small}} (n2);\n",
						"\\draw[bend right, ->] (n3)..controls +(right:2cm) and +(down:2cm)..(n3) node [pos=0.5,sloped, below]{value};\n",
						"\\draw[->]  (n3) to node [pos=0.25,sloped, above] {\\begin{small}value\\end{small}} (n4);\n"},
				{
						"\\draw[bend right,->]  (n4) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n0);\n",
						"\\draw[bend right,->]  (n4) to node [pos=.1, sloped, above] {\\begin{small}value\\end{small}} (n1);\n",
						"\\draw[bend right,->]  (n4) to node [pos=.15, sloped, above] {\\begin{small}value\\end{small}} (n2);\n",
						"\\draw[bend left,->]  (n4) to node [pos=.2, sloped, above] {\\begin{small}value\\end{small}} (n3);\n",
						"\\draw[bend left, ->] (n4)..controls +(right:2cm) and +(up:2cm)..(n4) node [pos=0.5,sloped, above]{value};\n"}};

		// String[][] arcSet = {
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"},
		// {"value", "value", "value", "value", "value", "value", "value",
		// "value", "value", "value", "value"}};

		return arcSet[fromNodeIndex][toNodeIndex].replace("value", value);
	}
}
