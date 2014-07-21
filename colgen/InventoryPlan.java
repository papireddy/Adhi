package colgen;

import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;
import input.Instance;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;

public class InventoryPlan {

	public double[][][]	qtyTran;
	public double		planCost;
	public double		reducedCost;

	public InventoryPlan(Instance instance) {
		planCost = 0;
		double[] toTranQty = new double[instance.getNoNodes()];
		double totalToTranQty = 0;
		qtyTran = new double[instance.getTime()][instance.getNoNodes()][instance.getNoNodes()];
		for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
			toTranQty[nodeIndex] = getQty(nodeIndex, instance);
			totalToTranQty += toTranQty[nodeIndex];
			planCost = planCost + getPlanCost(qtyTran[0][0][nodeIndex], instance, nodeIndex);
		}
		int prevNode = 0;
		for (int nodeIndex = 1; nodeIndex < instance.getNoNodes(); nodeIndex++) {
			qtyTran[0][prevNode][nodeIndex] = totalToTranQty;
			prevNode = nodeIndex;
			totalToTranQty = totalToTranQty - toTranQty[nodeIndex];
		}
	}

	private double getPlanCost(double qty, Instance instance, int node) {
		double cost = 0;
		for (int time = 0; time < instance.getTime(); time++) {
			cost = cost
					+ (qty + instance.getNode(node).getStartInv() - instance.getNode(node)
							.getDemand()[time]) * instance.getNode(node).getHoldingCost();
		}
		return cost;
	}

	private double getQty(int nodeIndex, Instance instance) {
		double qty = 0;
		double inv = instance.getNode(nodeIndex).getStartInv();
		for (int time = 0; time < instance.getTime(); time++) {
			inv = inv - instance.getNode(nodeIndex).getDemand()[time];
			qty = inv < qty ? inv : qty;
		}
		if (qty < 0) {
			return -qty;
		}
		return 0;
	}

	public InventoryPlan(double[][][] duals, Instance instance) {
		qtyTran = new double[instance.getTime()][instance.getNoNodes()][instance.getNoNodes()];
		try {
			IloCplex cplex = new IloCplex();
			IloNumVar qtyTranVar[][][] = makeQtyTranVar(cplex, instance);
			IloNumVar invVar[][] = makeInvVar(cplex, instance);

			// printDuals(duals);

			addConstraints(cplex, instance, qtyTranVar, invVar);

			// Adding Objective function
			IloNumExpr expr = cplex.constant(0);
			IloNumExpr holdExpr = cplex.constant(0);
			for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
				for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
					for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
						expr = cplex.sum(expr, cplex.prod(
								duals[timeIndex][fromNodeIndex][toNodeIndex],
								qtyTranVar[timeIndex][fromNodeIndex][toNodeIndex]));
					}
					if (fromNodeIndex != 0) {
						holdExpr = cplex.sum(holdExpr, cplex.prod(instance.getNode(fromNodeIndex)
								.getHoldingCost(), invVar[timeIndex][fromNodeIndex]));
					}
				}
			}
			cplex.addMinimize(cplex.sum(holdExpr, expr));
			// Done Adding Objective Function
			try {
				cplex.setOut(new FileOutputStream("E:\\InvOutput.txt"));
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
			if (cplex.solve()) {
				// System.out.println("Solved Inventory Plaln with ObjValue: " +
				// cplex.getObjValue());
				getQtyVars(cplex, qtyTranVar, instance);
				reducedCost = cplex.getObjValue();
				planCost = cplex.getValue(holdExpr);
			} else {
				System.err.println("Couldn't Solve Inventory SubProblem!!!\n Status:"
						+ cplex.getStatus());
			}

		} catch (IloException e) {
			e.printStackTrace();
		}
	}

	private void printDuals(double[][][] duals) {
		for (int i = 0; i < duals.length; i++) {
			for (int j = 0; j < duals[i].length; j++) {
				for (int k = 0; k < duals[i][j].length; k++) {
					System.out.print("duals[" + i + "][" + j + "][" + k + "]:" + duals[i][j][k]
							+ "; ");
				}
				System.out.println();
			}
		}
	}

	private void getQtyVars(IloCplex cplex, IloNumVar[][][] qtyTranVar, Instance instance)
			throws UnknownObjectException, IloException {
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					if (fromNodeIndex != toNodeIndex) {
						qtyTran[timeIndex][fromNodeIndex][toNodeIndex] = cplex
								.getValue(qtyTranVar[timeIndex][fromNodeIndex][toNodeIndex]);
						// if (qtyTran[timeIndex][fromNodeIndex][toNodeIndex] !=
						// 0) {
						// System.out.print("x^" + timeIndex + "_{" +
						// fromNodeIndex + ","
						// + toNodeIndex + "}:"
						// + qtyTran[timeIndex][fromNodeIndex][toNodeIndex] +
						// "; ");
						// }
					}
				}
			}
			// System.out.println();
		}
	}

	private void addConstraints(IloCplex cplex, Instance instance, IloNumVar[][][] qtyTranVar,
			IloNumVar[][] invVar) throws IloException {
		createInventoryBalanceConstraints(cplex, instance, qtyTranVar, invVar);
		createInventoryLimitConstraints(cplex, instance, invVar);

	}

	private void createInventoryLimitConstraints(IloCplex cplex, Instance instance,
			IloNumVar[][] invVar) throws IloException {
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				cplex.addGe(instance.getNode(nodeIndex).getMaxCapacity(),
						invVar[timeIndex][nodeIndex]);
				cplex.addLe(instance.getNode(nodeIndex).getMinCapacity(),
						invVar[timeIndex][nodeIndex]);
			}
		}
	}

	private void createInventoryBalanceConstraints(IloCplex cplex, Instance instance,
			IloNumVar[][][] qtyTranVar, IloNumVar[][] invVar) throws IloException {
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				IloNumExpr inExpr = cplex.constant(0);
				IloNumExpr outExpr = cplex.constant(0);
				for (int otherNodeIndex = 0; otherNodeIndex < instance.getNoNodes(); otherNodeIndex++) {
					inExpr = cplex.sum(inExpr, qtyTranVar[timeIndex][otherNodeIndex][nodeIndex]);
					outExpr = cplex.sum(outExpr, qtyTranVar[timeIndex][nodeIndex][otherNodeIndex]);
				}
				if (timeIndex == 0) {
					cplex.addEq(invVar[timeIndex + 1][nodeIndex], cplex.sum(
							cplex.constant(instance.getNode(nodeIndex).getStartInv()), inExpr,
							cplex.prod(-1, outExpr),
							cplex.constant(-instance.getNode(nodeIndex).getDemand()[timeIndex])));
				} else {
					cplex.addEq(invVar[timeIndex + 1][nodeIndex], cplex.sum(
							invVar[timeIndex][nodeIndex], inExpr, cplex.prod(-1, outExpr),
							cplex.constant(-instance.getNode(nodeIndex).getDemand()[timeIndex])));
				}
			}
		}
	}

	private IloNumVar[][][] makeQtyTranVar(IloCplex cplex, Instance instance) throws IloException {
		IloNumVar[][][] vars = new IloNumVar[instance.getTime()][][];
		for (int timeIndex = 0; timeIndex < instance.getTime(); timeIndex++) {
			vars[timeIndex] = new IloNumVar[instance.getNoNodes()][];
			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				vars[timeIndex][fromNodeIndex] = new IloNumVar[instance.getNoNodes()];
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					vars[timeIndex][fromNodeIndex][toNodeIndex] = cplex.numVar(0, Double.MAX_VALUE,
							IloNumVarType.Float, "x^" + timeIndex + "_{" + fromNodeIndex + ","
									+ toNodeIndex + "}");
				}
			}
		}
		return vars;
	}

	private IloNumVar[][] makeInvVar(IloCplex cplex, Instance instance) throws IloException {
		IloNumVar[][] vars = new IloNumVar[instance.getTime() + 1][];
		for (int timeIndex = 0; timeIndex < instance.getTime() + 1; timeIndex++) {
			vars[timeIndex] = new IloNumVar[instance.getNoNodes()];
			for (int nodeIndex = 0; nodeIndex < instance.getNoNodes(); nodeIndex++) {
				vars[timeIndex][nodeIndex] = cplex.numVar(0, Double.MAX_VALUE, IloNumVarType.Float,
						"I^" + timeIndex + "_" + nodeIndex);
			}
		}
		return vars;
	}
}
