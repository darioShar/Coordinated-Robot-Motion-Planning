
	/**
	 * Main program that takes as input a JSON storing the set of starting and target positions of robots
	 * and computes a solution to the coordinated motion problem, minimizing the 'makespan'
	 * 
	 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, dec 2020)
	 */
public class OptimizeMakespan {
	
	public static void main(String[] args) {
		System.out.println("Makespan optimization (CG:SHOP 2021 contest)\n");
		System.out.println(args.length);
		
		if(args.length<1) {
			System.out.println("Error: one argument required: input file in JSON format");
			System.exit(0);
		}

		String inputFile=args[0]; // input file storing the input instance
		System.out.println("Input file: "+inputFile);
		if(inputFile.endsWith(".json")==false) {
			System.out.println("Error: wrong input format");
			System.out.println("Supported input format: JSON format");
			System.exit(0);
		}

		Instance input=IO.loadInputInstance(inputFile); // read the input file
		System.out.println(input);

		MyBestAlgorithm algo=new MyBestAlgorithm(input); 
		algo.run(); // compute a solution for the input instance
		/*for (int i = 0; i < 10; i++)
			System.out.println(algo.updateTrajectory(input, i));*/
		
		Solution solution=algo.getSolution();
		System.out.println(solution); // print the statistics
		IO.saveSolutionToJSON(solution, input.name+"_makespan.json"); // export the solution in JSON format
	}

}
