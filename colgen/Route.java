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
	public short[][]	route;
	public double		routeCost;
	public double		reducedCost;

	public Route(int startNode, Instance instance, int type) {
		route = new short[instance.getNoNodes()][instance.getNoNodes()];
		if (type == 0) {// returns all 0 route or vehicle non going
			routeCost = 0;
		} else if (type == 1) {
			int prevNode = startNode;
			routeCost = 0;
			for (int nodeIndex = getNextUPNode(startNode, instance); nodeIndex != startNode; nodeIndex = getNextUPNode(
					nodeIndex, instance)) {
				route[prevNode][nodeIndex] = 1;
				routeCost = routeCost + instance.distanceMatrix[prevNode][nodeIndex]
						* instance.getVehicle(startNode).getPerUnitCost();
				prevNode = nodeIndex;
			}
			route[prevNode][startNode] = 1;
			routeCost = routeCost + instance.distanceMatrix[prevNode][startNode]
					* instance.getVehicle(startNode).getPerUnitCost();
		} else {
			int prevNode = startNode;
			routeCost = 0;
			for (int nodeIndex = getNextDOWNNode(startNode, instance); nodeIndex != startNode; nodeIndex = getNextDOWNNode(
					nodeIndex, instance)) {
				route[prevNode][nodeIndex] = 1;
				routeCost = routeCost + instance.distanceMatrix[prevNode][nodeIndex]
						* instance.getVehicle(startNode).getPerUnitCost();
				prevNode = nodeIndex;
			}
			route[prevNode][startNode] = 1;
			routeCost = routeCost + instance.distanceMatrix[prevNode][startNode]
					* instance.getVehicle(startNode).getPerUnitCost();
		}
	}

	private int getNextDOWNNode(int node, Instance instance) {
		if (node == 0) {
			return (instance.getNoNodes() - 1);
		}
		return (node - 1);
	}

	private int getNextUPNode(int node, Instance instance) {
		if (node == instance.getNoNodes() - 1) {
			return 0;
		}
		return ++node;
	}

	// ROUTE with duals
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
					costExpr = cplex.sum(costExpr, cplex.prod(
							instance.getVehicle(startNode).getPerUnitCost()
									* instance.distanceMatrix[fromNodeIndex][toNodeIndex],
							routeVars[fromNodeIndex][toNodeIndex]));

					expr = cplex.sum(expr, cplex.prod(instance.getVehicle(startNode)
							.getVehicleCapacity() * duals[fromNodeIndex][toNodeIndex],
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
				// System.out.println("Solved Route Sub Problem for " +
				// startNode + " node ObjVal:"
				// + cplex.getObjValue());
				getRoute(cplex, routeVars, instance);
				reducedCost = cplex.getObjValue();
				routeCost = cplex.getValue(costExpr);
			} else {
				System.err.println("Couldn't Solve Routing SubProblem for " + startNode
						+ " node!!!\n Status:" + cplex.getStatus());
			}

		} catch (IloException e) {
			e.printStackTrace();
		}
	}

	private void getRoute(IloCplex cplex, IloNumVar[][] routeVars, Instance instance)
			throws UnknownObjectException, IloException {
		for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
			for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
				route[fromNodeIndex][toNodeIndex] = (short) cplex
						.getValue(routeVars[fromNodeIndex][toNodeIndex]);
				// System.out.print("route[" + fromNodeIndex + "][" +
				// toNodeIndex
				// + "]:" + route[fromNodeIndex][toNodeIndex] + "; ");
			}
		}
		// System.out.println();
	}

	private void addConstraints(IloCplex cplex, Instance instance, int startNode,
			IloNumVar[][] routeVars) throws IloException {
		createStartEndNodeConstraints(cplex, startNode, instance, routeVars);
		createRouteBalanceConstraints(cplex, instance, routeVars);

	}

	private void createRouteBalanceConstraints(IloCplex cplex, Instance instance,
			IloNumVar[][] routeVars) throws IloException {
		for (int fromNodeIndex = 0; fromNodeIndex < instance.getNoNodes(); fromNodeIndex++) {
			IloNumExpr outExpr = cplex.constant(0);
			IloNumExpr inExpr = cplex.constant(0);
			for (int toNodeIndex = 0; toNodeIndex < instance.getNoNodes(); toNodeIndex++) {
				outExpr = cplex.sum(outExpr, routeVars[fromNodeIndex][toNodeIndex]);
				inExpr = cplex.sum(inExpr, routeVars[toNodeIndex][fromNodeIndex]);
			}
			cplex.addEq(inExpr, outExpr);
		}
	}

	private void createStartEndNodeConstraints(IloCplex cplex, int startNode, Instance instance,
			IloNumVar[][] routeVars) throws IloException {
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
				vars[fromNodeIndex][toNodeIndex] = cplex.numVar(0, 1, IloNumVarType.Bool, "y_{"
						+ fromNodeIndex + "," + toNodeIndex + "}");
			}
		}
		return vars;
	}
}
