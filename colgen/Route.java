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

public class Route {
	public short[][] route;
	public double routeCost;
	public double reducedCost;
	public Route(double[][] duals, int startNode, Instance instance) {
		route = new short[instance.getNoNodes()][instance.getNoNodes()];
		try {
			IloCplex cplex = new IloCplex();

			IloNumVar[][] routeVars = createRouteVariables(cplex, instance);

			addConstraints(cplex, instance, startNode, routeVars);

			// Adding Objective
			IloNumExpr costExpr = cplex.constant(0);
			IloNumExpr expr = cplex.constant(0);

			for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
				for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
					costExpr = cplex
							.sum(costExpr,
									cplex.prod(
											instance.getVehicle(startNode)
													.getPerUnitCost()
													* instance.distanceMatrix[fromNodeIndex][toNodeIndex],
											routeVars[fromNodeIndex][toNodeIndex]));

					expr = cplex.sum(expr, cplex.prod(
							instance.getVehicle(startNode).getVehicleCapacity()
									* duals[fromNodeIndex][toNodeIndex],
							routeVars[fromNodeIndex][toNodeIndex]));

				}
			}

			cplex.addMinimize(cplex.sum(costExpr, cplex.prod(-1, expr)));
			// DONE Adding Objective

			try {
				cplex.setOut(new FileOutputStream("E:\\RouteOutput.txt"));
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}

			if (cplex.solve()) {
				System.out.println("Solved:" + cplex.getObjValue());
				getRoute(cplex, routeVars, instance);
				reducedCost = cplex.getObjValue();
				routeCost = cplex.getValue(costExpr);
			} else {
				System.err.println("Couldn't Solve Routing SubProblem for "
						+ startNode + " node!!!\n Status:" + cplex.getStatus());
			}

		} catch (IloException e) {
			e.printStackTrace();
		}
	}
	private void getRoute(IloCplex cplex, IloNumVar[][] routeVars,
			Instance instance) throws UnknownObjectException, IloException {
		for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
			for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
				route[fromNodeIndex][toNodeIndex] = (short) cplex
						.getValue(routeVars[fromNodeIndex][toNodeIndex]);
				// System.out.print("route[" + fromNodeIndex + "][" +
				// toNodeIndex
				// + "]:" + route[fromNodeIndex][toNodeIndex] + "; ");
			}
		}
		System.out.println();
	}
	private void addConstraints(IloCplex cplex, Instance instance,
			int startNode, IloNumVar[][] routeVars) throws IloException {
		createStartEndNodeConstraints(cplex, startNode, instance, routeVars);
		createRouteBalanceConstraints(cplex, instance, routeVars);

	}

	private void createRouteBalanceConstraints(IloCplex cplex,
			Instance instance, IloNumVar[][] routeVars) throws IloException {
		for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
			IloNumExpr outExpr = cplex.constant(0);
			IloNumExpr inExpr = cplex.constant(0);
			for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
				outExpr = cplex.sum(outExpr,
						routeVars[fromNodeIndex][toNodeIndex]);
				inExpr = cplex.sum(inExpr,
						routeVars[toNodeIndex][fromNodeIndex]);
			}
			cplex.addEq(inExpr, outExpr);
		}
	}

	private void createStartEndNodeConstraints(IloCplex cplex, int startNode,
			Instance instance, IloNumVar[][] routeVars) throws IloException {
		IloNumExpr outExpr = cplex.constant(0);
		IloNumExpr inExpr = cplex.constant(0);
		for (int otherNodeIndex = 0; otherNodeIndex < instance.getNoNodes(); otherNodeIndex++) {
			outExpr = cplex.sum(outExpr, routeVars[startNode][otherNodeIndex]);
			inExpr = cplex.sum(inExpr, routeVars[otherNodeIndex][startNode]);
		}
		cplex.addEq(outExpr, 1);
		cplex.addEq(inExpr, 1);
	}
	private IloNumVar[][] createRouteVariables(IloCplex cplex, Instance instance)
			throws IloException {
		IloNumVar[][] vars = new IloNumVar[instance.getNoNodes()][];
		for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
			vars[fromNodeIndex] = new IloNumVar[instance.getNoNodes()];
			for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
				vars[fromNodeIndex][toNodeIndex] = cplex.numVar(0, 1,
						IloNumVarType.Bool, "y_{" + fromNodeIndex + ","
								+ toNodeIndex + "}");
			}
		}
		return vars;
	}
}
