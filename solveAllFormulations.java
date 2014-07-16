public class solveAllFormulations {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		try {
			solveMIRPTQtyVeh.main(args);
			solveMIRPTvisitVar.main(args);
			solveMIRPTQtyInout.main(args);
			solveMIRPTVehPathQty.main(args);
		} catch (Exception e) {
			e.printStackTrace();
		}
		System.out.println("All Formulations Done!!!");
	}

}
