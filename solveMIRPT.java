import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import input.Instance;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class solveMIRPT {

	public static BufferedWriter bw = null;
	public static BufferedWriter solwriter = null;
	public static OutputStream logwriter = null;

	public static void main(String[] args) {
		String inputfile = "D:\\The Folder\\IRP\\TestCases\\Problems";
		try {
			BufferedWriter activitywriter = new BufferedWriter(new FileWriter(
					"D:\\The Folder\\IRP\\TestCases\\Activity\\activity"
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
		System.out.println("Multi Formulation Done!!!");
	}

	static void solveProblems(String inputfile) {
		File file = new File(inputfile);
		if (file.isDirectory()) {
			File[] list = file.listFiles();
			for (int listIndex = 0; listIndex < list.length; listIndex++) {
				solveProblems(list[listIndex].getAbsolutePath());
			}
		} else {
			if (file.isFile()) {
				String fileName = file.getAbsolutePath();
				long startTime;
				long endTime;
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
					bw = new BufferedWriter(
							new FileWriter(
									getTargetLoc(fileName)
											+ "\\"
											+ fileName.substring(
													fileName.lastIndexOf("\\") + 1,
													fileName.indexOf("."))
											+ "-Multi-"
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
											+ "-Multi-"
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
									+ "-Multi-"
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
					// + "-Multi-"
					// + slp
					// + scuts
					// + (new SimpleDateFormat(
					// "MMM-dd--HH-mm-ss"))
					// .format(new Date())
					// + "-Map.tex"));

					// processTheProblem(instance, lp, cuts);
					// processProblem3dqty(instance, lp, cuts);
					processProblem6dqty(instance, lp, cuts);

					try {
						solwriter.close();
						bw.close();
					} catch (IOException e) {
						e.printStackTrace();
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}

	private static void processProblem6dqty(Instance instance, boolean lp,
			boolean cuts) {
		try {
			IloCplex cplex = new IloCplex();
			IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
			IloNumVar[][][] qtyVar = createQuantity3dVariables(cplex, instance);

			IloNumVar[][][][][][] intenQtyVar = createIntendQtyTran(cplex,
					instance);
			IloNumVar[][][][] routeVar = null;
			if (lp) {
				routeVar = createRouteVariablesLP(cplex, instance);
			} else {
				routeVar = createRouteVariablesIP(cplex, instance);
			}

			create6dConstraints(cplex, instance, cuts, invVar, qtyVar,
					routeVar, intenQtyVar);

			createObjective(cplex, instance, routeVar, invVar);

			setParameters(cplex);

			if (cplex.solve()) {
				System.out.println("Solved:" + cplex.getObjValue());
				get6DSolution(cplex, instance, invVar, qtyVar, routeVar,
						intenQtyVar);
			} else {
				System.out.println("Couldn't Solve!!" + cplex.getStatus());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	private static void create6dConstraints(IloCplex cplex, Instance instance,
			boolean cuts, IloNumVar[][] invVar, IloNumVar[][][] qtyVar,
			IloNumVar[][][][] routeVar, IloNumVar[][][][][][] intenQtyVar) {
		try {
			createInvBalance6d(instance, cplex, qtyVar, invVar);
			createRouteBalConstraints(cplex, instance, routeVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createSubTourElimination(cplex, instance, qtyVar, intenQtyVar);
			createRouIntenQuanLinkConstraints(cplex, instance, intenQtyVar,
					routeVar);
			createOneVehiclePerNodeConstraints(cplex, instance, routeVar);
			createVehicleStartNodeConstraints(cplex, instance, routeVar);
			createMaxDistanceConstraints(cplex, instance, routeVar);

			if (cuts) {
				// Min Visit Cuts
				createMinNumberVisitCuts(cplex, instance, routeVar);
				IloRange[] visitCuts = getMinNumberVisitCuts(cplex, instance,
						routeVar);
				cplex.addUserCuts(visitCuts);
				createDemandInvDiffCuts(cplex, instance, routeVar, invVar);

				// IDD cuts
				createStartInvDemDiffCuts(cplex, instance, routeVar);

				// // eq15 Cuts
				createEq15Cuts(cplex, instance, routeVar, invVar);

				// eq14 Cuts
				Object[] eq14cuts = getEq14Cuts(cplex, instance, routeVar,
						invVar);
				for (int i = 0; i < eq14cuts.length; i++) {
					cplex.addUserCut((IloConstraint) eq14cuts[i]);
				}

				// createEq14Cuts(cplex, instance, routeVar, invVar);
			}

		} catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	@SuppressWarnings("rawtypes")
	private static Object[] getEq14Cuts(IloCplex cplex, Instance instance,
			IloNumVar[][][][] routeVar, IloNumVar[][] invVar)
			throws IloException, IOException {

		ArrayList<IloConstraint> cuts = new ArrayList<IloConstraint>();
		int count = 0;
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
						cuts.add(
								count++,
								cplex.ge(invVar[timeIndex][nodeIndex],
										cplex.prod(demandSum, expr)));
					} else if (demandSum < 0) {
						cuts.add(count++, cplex.ge(cplex.sum(
								instance.getNode(nodeIndex).getMaxCapacity(),
								cplex.prod(-1, invVar[timeIndex][nodeIndex])),
								cplex.prod(-demandSum, expr)));
					}

				}
			}
		}
		return cuts.toArray();
	}

	private static IloRange[] getMinNumberVisitCuts(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Minumum Visit Cuts:");
		bw.newLine();
		IloRange[] cuts = new IloRange[10000];
		int cutcount = 0;
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
					cuts[cutcount++] = cplex.ge(
							expr,
							getMinimumVisits(instance, fromTimeIndex,
									toTimeIndex, nodeIndex));
					System.out.println(cplex.ge(
							expr,
							getMinimumVisits(instance, fromTimeIndex,
									toTimeIndex, nodeIndex)));

				}
			}
		}
		return cuts;
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

		for (int timeIndex = 1; timeIndex < instance.getTime(); timeIndex++) {
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
				bw.write(cplex.addGe(
						invVar[timeIndex - 1][nodeIndex],
						cplex.prod(
								instance.getNode(nodeIndex).getDemand()[timeIndex],
								cplex.sum(1, cplex.prod(-1, expr))))
						+ "");
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

	private static void createInvBalance6d(Instance instance, IloCplex cplex,
			IloNumVar[][][] qtyVar, IloNumVar[][] invVar) throws IloException,
			IOException {
		bw.write("Inventory Balance 3DQty Constraints:");
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

	private static void createOneVehiclePerNodeConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Max One Vehicle per Node Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					expr = cplex
							.sum(expr,
									routeVar[timeIndex][vehicleIndex][vehicleIndex][nodeIndex]);
				}
				bw.write(cplex.addLe(expr, 1) + "\n");
			}
		}
	}

	private static void createSubTourElimination(IloCplex cplex,
			Instance instance, IloNumVar[][][] qtyVar,
			IloNumVar[][][][][][] intenQtyVar) throws IloException, IOException {
		bw.write("Subtour Elimination Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < intenQtyVar.length; timeIndex++) {
			for (int intFromIndex = 0; intFromIndex < instance.getNoNodes(); intFromIndex++) {
				for (int intToIndex = 0; intToIndex < instance.getNoNodes(); intToIndex++) {
					if (intFromIndex == intToIndex) {
						continue;
					}
					for (int focusNodeIndex = 0; focusNodeIndex < instance
							.getNoNodes(); focusNodeIndex++) {

						if (intFromIndex == focusNodeIndex) {
							IloNumExpr toExpr = cplex.constant(0);
							IloNumExpr fromExpr = cplex.constant(0);

							for (int otherNodeIndex = 0; otherNodeIndex < instance
									.getNoNodes(); otherNodeIndex++) {

								for (int vehicleIndex = 0; vehicleIndex < instance
										.getNoNodes(); vehicleIndex++) {
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
								}
							}
							bw.write(cplex.addEq(
									cplex.sum(fromExpr, cplex.prod(-1, toExpr)),
									qtyVar[timeIndex][intFromIndex][intToIndex])
									+ "");
							bw.newLine();

						} else if (intToIndex == focusNodeIndex) {
							IloNumExpr toExpr = cplex.constant(0);
							IloNumExpr fromExpr = cplex.constant(0);

							for (int vehicleIndex = 0; vehicleIndex < instance
									.getNoNodes(); vehicleIndex++) {
								for (int otherNodeIndex = 0; otherNodeIndex < instance
										.getNoNodes(); otherNodeIndex++) {
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
								}
							}
							bw.write(cplex.addEq(
									cplex.sum(fromExpr, cplex.prod(-1, toExpr)),
									cplex.prod(
											-1,
											qtyVar[timeIndex][intFromIndex][intToIndex]))
									+ "");
							bw.newLine();

						} else {
							for (int vehicleIndex = 0; vehicleIndex < instance
									.getNoNodes(); vehicleIndex++) {
								IloNumExpr fromExpr = cplex.constant(0);
								IloNumExpr toExpr = cplex.constant(0);
								for (int otherNodeIndex = 0; otherNodeIndex < instance
										.getNoNodes(); otherNodeIndex++) {
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
								}
								bw.write(cplex.addEq(fromExpr, toExpr) + "");
								bw.newLine();
							}
						}
					}
				}
			}
		}
	}

	private static void createSubTourElimination(IloCplex cplex,
			Instance instance, IloNumVar[][][][] qtyVar,
			IloNumVar[][][][][][] intenQtyVar) throws IloException, IOException {
		bw.write("Subtour Elimination Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < intenQtyVar.length; timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int intFromIndex = 0; intFromIndex < instance.getNoNodes(); intFromIndex++) {
					for (int intToIndex = 0; intToIndex < instance.getNoNodes(); intToIndex++) {
						for (int focusNodeIndex = 0; focusNodeIndex < instance
								.getNoNodes(); focusNodeIndex++) {
							if (intFromIndex == focusNodeIndex) {
								IloNumExpr toExpr = cplex.constant(0);
								IloNumExpr fromExpr = cplex.constant(0);

								for (int otherNodeIndex = 0; otherNodeIndex < instance
										.getNoNodes(); otherNodeIndex++) {
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
								}
								bw.write(cplex.addEq(
										cplex.sum(fromExpr,
												cplex.prod(-1, toExpr)),
										qtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex])
										+ "");
								bw.newLine();

							} else if (intToIndex == focusNodeIndex) {
								IloNumExpr toExpr = cplex.constant(0);
								IloNumExpr fromExpr = cplex.constant(0);
								for (int otherNodeIndex = 0; otherNodeIndex < instance
										.getNoNodes(); otherNodeIndex++) {
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
								}
								bw.write(cplex.addEq(
										cplex.sum(fromExpr,
												cplex.prod(-1, toExpr)),
										cplex.prod(
												-1,
												qtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex]))
										+ "");
								bw.newLine();

							} else {
								IloNumExpr fromExpr = cplex.constant(0);
								IloNumExpr toExpr = cplex.constant(0);
								for (int otherNodeIndex = 0; otherNodeIndex < instance
										.getNoNodes(); otherNodeIndex++) {
									fromExpr = cplex
											.sum(fromExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][focusNodeIndex][otherNodeIndex]);
									toExpr = cplex
											.sum(toExpr,
													intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][otherNodeIndex][focusNodeIndex]);
								}
								bw.write(cplex.addEq(fromExpr, toExpr) + "");
								bw.newLine();
							}
						}
					}
				}
			}
		}
	}

	private static void createRouIntenQuanLinkConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][][][] intenQtyVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Route Quantity Link Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int rouFromIndex = 0; rouFromIndex < instance.getNoNodes(); rouFromIndex++) {
					for (int rouToIndex = 0; rouToIndex < instance.getNoNodes(); rouToIndex++) {
						IloNumExpr expr = cplex.constant(0);
						for (int intFromIndex = 0; intFromIndex < instance
								.getNoNodes(); intFromIndex++) {
							for (int intToIndex = 0; intToIndex < instance
									.getNoNodes(); intToIndex++) {
								expr = cplex
										.sum(expr,
												intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex][rouToIndex]);
							}
						}
						bw.write(cplex.addLe(
								expr,
								cplex.prod(
										instance.getVehicle(vehicleIndex)
												.getVehicleCapacity(),
										routeVar[timeIndex][vehicleIndex][rouFromIndex][rouToIndex]))
								+ "");
						bw.newLine();
					}
				}
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

	private static IloNumVar[][][] createQuantity3dVariables(IloCplex cplex,
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

	private static IloNumVar[][][][][][] createIntendQtyTran(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar[][][][][][] intQtyTranVar = new IloNumVar[instance.getTime()][][][][][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			intQtyTranVar[timeIndex] = new IloNumVar[instance.getNoNodes()][][][][];
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				intQtyTranVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()][][][];
				for (int intFromIndex = 0; intFromIndex < instance.getNoNodes(); intFromIndex++) {
					intQtyTranVar[timeIndex][vehicleIndex][intFromIndex] = new IloNumVar[instance
							.getNoNodes()][][];
					for (int intToIndex = 0; intToIndex < instance.getNoNodes(); intToIndex++) {
						intQtyTranVar[timeIndex][vehicleIndex][intFromIndex][intToIndex] = new IloNumVar[instance
								.getNoNodes()][];
						for (int rouFromIndex = 0; rouFromIndex < instance
								.getNoNodes(); rouFromIndex++) {
							intQtyTranVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex] = new IloNumVar[instance
									.getNoNodes()];
							for (int rouToIndex = 0; rouToIndex < instance
									.getNoNodes(); rouToIndex++) {
								intQtyTranVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex][rouToIndex] = cplex
										.numVar(0, Float.MAX_VALUE,
												IloNumVarType.Float, "x^{"
														+ timeIndex + ","
														+ vehicleIndex + "}_{"
														+ intFromIndex + ","
														+ intToIndex + ","
														+ rouFromIndex + ","
														+ rouToIndex + "}");
							}
						}
					}
				}
			}
		}
		return intQtyTranVar;
	}

	private static IloNumVar[][][][] createRouteVariablesIP(IloCplex cplex,
			Instance instance) throws IloException, IOException {
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
								.numVar(0, 1, IloNumVarType.Bool, "y^{"
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
			Instance instance) throws IloException, IOException {
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
								.numVar(0, 1, IloNumVarType.Float, "y^{"
										+ timeIndex + "," + vehicleIndex + "}"
										+ "_{" + fromIndex + "," + toIndex
										+ "}");
					}
				}
			}
		}
		return rouVar;
	}

	private static String getTargetLoc(String fileName) {
		String lastfolder = fileName.substring(0, fileName.lastIndexOf("."));
		lastfolder = lastfolder.replace("TestCases", "TestCases\\FormMulti\\");
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

	private static void get6DSolution(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar, IloNumVar[][][] qtyVar,
			IloNumVar[][][][] routeVar, IloNumVar[][][][][][] intenQtyVar) {
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
											+ ";");
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

			solwriter.write("Intended Quantity Variables:\n");
			solwriter.flush();

			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int intFromIndex = 0; intFromIndex < instance
							.getNoNodes(); intFromIndex++) {
						for (int intToIndex = 0; intToIndex < instance
								.getNoNodes(); intToIndex++) {
							for (int rouFromIndex = 0; rouFromIndex < instance
									.getNoNodes(); rouFromIndex++) {
								for (int rouToIndex = 0; rouToIndex < instance
										.getNoNodes(); rouToIndex++) {
									if (cplex
											.getValue(intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex][rouToIndex]) != 0) {
										solwriter
												.write(intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex][rouToIndex]
														.getName()
														+ ":"
														+ cplex.getValue(intenQtyVar[timeIndex][vehicleIndex][intFromIndex][intToIndex][rouFromIndex][rouToIndex]));
										solwriter.write("; ");
									}
								}
							}
						}
					}
				}
				solwriter.newLine();
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void createConstraints3DQty(Instance instance,
			IloCplex cplex, IloNumVar[][][] qtyVar, IloNumVar[][][][] routeVar,
			IloNumVar[][] invVar, IloNumVar[][][] nextQtyVar) {
		try {
			createInvBalance3d(instance, cplex, qtyVar, invVar);
			createRouteBalConstraints(cplex, instance, routeVar);
			createCapCon3d(cplex, instance, qtyVar, routeVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createOneVehiclePerNodeConstraints(cplex, instance, routeVar);
			createVehicleStartNodeConstraints(cplex, instance, routeVar);
			// createSubTourEliminationConstraints(cplex, instance, nextQtyVar,
			// routeVar);
			bw.flush();
		} catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static void createSubTourEliminationConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][] nextQtyVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Subtour Elimination Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
					if (vehicleIndex == fromIndex)
						continue;
					for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
						if (fromIndex == toIndex)
							continue;
						bw.write(cplex.addLe(
								cplex.sum(
										nextQtyVar[timeIndex][vehicleIndex][fromIndex],
										cplex.prod(
												-1,
												nextQtyVar[timeIndex][vehicleIndex][toIndex]),
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												routeVar[timeIndex][vehicleIndex][fromIndex][toIndex])),
								instance.getVehicle(vehicleIndex)
										.getVehicleCapacity()
										- instance.getNode(toIndex).getDemand()[timeIndex])
								+ "");
						bw.newLine();
					}
					bw.write(cplex.addLe(instance.getNode(vehicleIndex)
							.getDemand()[timeIndex],
							nextQtyVar[timeIndex][vehicleIndex][fromIndex])
							+ "");
					bw.newLine();
					bw.write(cplex.addGe(
							nextQtyVar[timeIndex][vehicleIndex][fromIndex],
							instance.getVehicle(vehicleIndex)
									.getVehicleCapacity())
							+ "");
					bw.newLine();
				}

			}
		}
	}

	private static void createCapCon3d(IloCplex cplex, Instance instance,
			IloNumVar[][][] qtyVar, IloNumVar[][][][] routeVar)
			throws IloException, IOException {
		bw.write("Quantity Capcity Link Constraints:");
		bw.newLine();

		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
				for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {

					IloNumExpr expr = cplex.constant(0);
					for (int vehicleIndex = 0; vehicleIndex < instance
							.getNoNodes(); vehicleIndex++) {
						expr = cplex
								.sum(expr,
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												routeVar[timeIndex][vehicleIndex][fromIndex][toIndex]));
					}
					bw.write(cplex.addLe(qtyVar[timeIndex][fromIndex][toIndex],
							expr) + "");
					bw.newLine();
				}
			}
		}
	}

	private static void createInvBalance3d(Instance instance, IloCplex cplex,
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

	private static IloNumVar[][][] createNextQtyVar(IloCplex cplex,
			Instance instance) throws IloException, IOException {
		IloNumVar[][][] nextQtyVar = new IloNumVar[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			nextQtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()][];
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				nextQtyVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()];
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					nextQtyVar[timeIndex][vehicleIndex][nodeIndex] = cplex
							.numVar(0, Float.MAX_VALUE, IloNumVarType.Float,
									"u^{" + timeIndex + "," + vehicleIndex
											+ "}_" + nodeIndex);
					bw.write(nextQtyVar[timeIndex][vehicleIndex][nodeIndex]
							+ "\t");
				}
				bw.newLine();
			}
		}
		return nextQtyVar;
	}

	// private static void processTheProblem(Instance instance) {
	// try {
	// IloCplex cplex = new IloCplex();
	//
	// IloNumVar[][][][] qtyVar = createQuantityVariables(cplex, instance);
	// IloNumVar[][][][] routeVar = createRouteVariables(cplex, instance);
	// IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
	//
	// createConstraints(cplex, instance, qtyVar, routeVar, invVar);
	// createObjective(cplex, instance, routeVar, invVar);
	//
	// if (cplex.solve()) {
	// System.out.println("Solved:" + cplex.getObjValue());
	// getSolution(cplex, instance, qtyVar, routeVar, invVar);
	// }
	//
	// } catch (Exception e) {
	// e.printStackTrace();
	// }
	// }

	private static void getSolution(IloCplex cplex, Instance instance,
			IloNumVar[][][][] qtyVar, IloNumVar[][][][] routeVar,
			IloNumVar[][] invVar) {
		try {
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
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {
							if (cplex
									.getValue(qtyVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0) {
								solwriter
										.write(qtyVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(qtyVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ ";");
							}
						}
						solwriter.write("\t\t\t");
					}
					solwriter.newLine();
				}
				solwriter.write("************************");
				solwriter.newLine();
			}

			solwriter.write("Route Variables:");
			solwriter.newLine();
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int fromNodeIndex = 0; fromNodeIndex < instance
							.getNoNodes(); fromNodeIndex++) {
						for (int toNodeIndex = 0; toNodeIndex < instance
								.getNoNodes(); toNodeIndex++) {
							if (cplex
									.getValue(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]) != 0)
								solwriter
										.write(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex]
												.getName()
												+ ":"
												+ cplex.getValue(routeVar[timeIndex][vehicleIndex][fromNodeIndex][toNodeIndex])
												+ "; ");
						}
						solwriter.write("\t\t\t");
					}
					solwriter.newLine();
				}
				solwriter.write("************************");
				solwriter.newLine();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void createConstraints(IloCplex cplex, Instance instance,
			IloNumVar[][][][] qtyVar, IloNumVar[][][][] routeVar,
			IloNumVar[][] invVar) {
		try {
			createInvBalConstraints(cplex, instance, qtyVar, invVar);
			createCapLimitConstraints(cplex, instance, qtyVar, routeVar);
			createMinInvConstraints(cplex, instance, invVar);
			createMaxInvConstraints(cplex, instance, invVar);
			createRouteBalConstraints(cplex, instance, routeVar);
			createVehicleBaseLocConstraints(cplex, instance, routeVar);
			createVehicleStartConstraints(cplex, instance, routeVar);
			createMaxDistanceConstraints(cplex, instance, routeVar);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void createVehicleStartConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Vehicle Starting Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr expr = cplex.constant(0);
				for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						expr = cplex
								.sum(expr,
										routeVar[timeIndex][vehicleIndex][nodeIndex][otherNodeIndex]);
					}
				}
				expr = cplex.prod(instance.getVehicle(nodeIndex)
						.getVehicleCapacity(), expr);
				bw.write(cplex.addGe(cplex.sum(instance.getNode(nodeIndex)
						.getDemand()[timeIndex], expr), 0)
						+ "");
				bw.newLine();
			}
		}

	}
	private static void createVehicleBaseLocConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] routeVar) throws IloException,
			IOException {
		bw.write("Vehicle Base Location Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
					IloNumExpr expr = cplex.constant(0);
					for (int otherNodeIndex = 0; otherNodeIndex < instance
							.getNoNodes(); otherNodeIndex++) {
						expr = cplex
								.sum(expr,
										routeVar[timeIndex][vehicleIndex][nodeIndex][otherNodeIndex]);
					}
					if (vehicleIndex == nodeIndex) {
						bw.write(cplex.addLe(expr, 1) + "");
						bw.newLine();
					}
					// else {
					// bw.write(cplex.addEq(expr, 0) + "");
					// bw.newLine();
					// }
				}
			}
		}
	}

	private static void createCapLimitConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] qtyVar,
			IloNumVar[][][][] routeVar) throws IloException, IOException {
		bw.write("Capacity Limit Constraints:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int vehicleIndex = 0; vehicleIndex < instance.getNoNodes(); vehicleIndex++) {
				for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
					for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
						bw.write(cplex
								.addLe(qtyVar[timeIndex][vehicleIndex][fromIndex][toIndex],
										cplex.prod(
												instance.getVehicle(
														vehicleIndex)
														.getVehicleCapacity(),
												routeVar[timeIndex][vehicleIndex][fromIndex][toIndex]))
								+ "");
						bw.newLine();
					}
				}
			}
		}
	}

	private static void createInvBalConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][][][] qtyVar, IloNumVar[][] invVar)
			throws IloException, IOException {
		bw.write("Inventory Balance at the nodes:");
		bw.newLine();
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr outExpr = cplex.constant(0);
				IloNumExpr inExpr = cplex.constant(0);
				for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {
					for (int vehicleIndex = 0; vehicleIndex < instance
							.getNoNodes(); vehicleIndex++) {
						outExpr = cplex
								.sum(outExpr,
										qtyVar[timeIndex][vehicleIndex][nodeIndex][toIndex]);
						inExpr = cplex
								.sum(inExpr,
										qtyVar[timeIndex][vehicleIndex][toIndex][nodeIndex]);
					}
				}

				bw.write(cplex.addEq(invVar[timeIndex + 1][nodeIndex], cplex
						.sum(invVar[timeIndex][nodeIndex], inExpr, cplex.prod(
								-1, outExpr), cplex.constant(instance.getNode(
								nodeIndex).getDemand()[timeIndex])))
						+ "");
				bw.newLine();
			}
		}
	}

	private static IloNumVar[][][][] createQuantityVariables(IloCplex cplex,
			Instance instance) throws IloException {
		IloNumVar[][][][] qtyVar = new IloNumVar[instance.getTime()][][][];
		for (int timeIndex = 0; timeIndex < qtyVar.length; timeIndex++) {
			qtyVar[timeIndex] = new IloNumVar[instance.getNoNodes()][][];
			for (int vehicleIndex = 0; vehicleIndex < qtyVar[timeIndex].length; vehicleIndex++) {
				qtyVar[timeIndex][vehicleIndex] = new IloNumVar[instance
						.getNoNodes()][];
				for (int fromIndex = 0; fromIndex < qtyVar[timeIndex][vehicleIndex].length; fromIndex++) {
					qtyVar[timeIndex][vehicleIndex][fromIndex] = new IloNumVar[instance
							.getNoNodes()];
					for (int toIndex = 0; toIndex < qtyVar[timeIndex][vehicleIndex][fromIndex].length; toIndex++) {

						qtyVar[timeIndex][vehicleIndex][fromIndex][toIndex] = cplex
								.numVar(0, Double.MAX_VALUE,
										IloNumVarType.Float, "q^{" + timeIndex
												+ "," + vehicleIndex + "}"
												+ "_{" + fromIndex + ","
												+ toIndex + "}");
					}
				}
			}
		}
		return qtyVar;
	}

	private static void processProblem3dqty(Instance instance, boolean lp,
			boolean cuts) {
		try {
			IloCplex cplex = new IloCplex();

			IloNumVar[][][] qtyVar = createQuantity3dVariables(cplex, instance);
			IloNumVar[][][][] routeVar = null;
			IloNumVar[][] invVar = createInventoryVariables(cplex, instance);
			IloNumVar[][][] nextQtyVar = createNextQtyVar(cplex, instance);
			if (lp) {
				routeVar = createRouteVariablesLP(cplex, instance);
			} else {
				routeVar = createRouteVariablesIP(cplex, instance);
			}

			createConstraints3DQty(instance, cplex, qtyVar, routeVar, invVar,
					nextQtyVar);

			createObjective(cplex, instance, routeVar, invVar);
			cplex.exportModel("E:\\MIRPT.mps");

			if (cplex.solve()) {
				System.out.println("Solved:" + cplex.getObjValue());
				for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
					for (int fromIndex = 0; fromIndex < instance.getNoNodes(); fromIndex++) {
						for (int toIndex = 0; toIndex < instance.getNoNodes(); toIndex++) {

							IloNumExpr expr = cplex.constant(0);
							for (int vehicleIndex = 0; vehicleIndex < instance
									.getNoNodes(); vehicleIndex++) {
								expr = cplex
										.sum(expr,
												cplex.prod(
														instance.getVehicle(
																vehicleIndex)
																.getVehicleCapacity(),
														routeVar[timeIndex][vehicleIndex][fromIndex][toIndex]));
							}
						}
					}
				}
				get3DSolution(cplex, instance, qtyVar, routeVar, invVar,
						nextQtyVar);

			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void get3DSolution(IloCplex cplex, Instance instance,
			IloNumVar[][][] qtyVar, IloNumVar[][][][] routeVar,
			IloNumVar[][] invVar, IloNumVar[][][] nextQtyVar) {
		try {
			solwriter.write("Inventory Variables:");
			solwriter.newLine();
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
					solwriter.newLine();
				}
				solwriter.write("************************");
				solwriter.newLine();
			}

			solwriter.write("Route Variables:");
			solwriter.newLine();
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

			for (int timeIndex = 0; timeIndex < nextQtyVar.length; timeIndex++) {
				for (int vehicleIndex = 0; vehicleIndex < nextQtyVar[timeIndex].length; vehicleIndex++) {
					for (int nodeIndex = 0; nodeIndex < nextQtyVar[timeIndex][vehicleIndex].length; nodeIndex++) {
						if (cplex
								.getValue(nextQtyVar[timeIndex][vehicleIndex][nodeIndex]) != 0) {
							solwriter
									.write(nextQtyVar[timeIndex][vehicleIndex][nodeIndex]
											.getName()
											+ ":"
											+ cplex.getValue(nextQtyVar[timeIndex][vehicleIndex][nodeIndex])
											+ "; ");
						}

					}
					solwriter.newLine();
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
