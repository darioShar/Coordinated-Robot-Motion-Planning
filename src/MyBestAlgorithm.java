import java.util.ArrayList;
import java.util.Collections;

import java.lang.*;
import java.util.HashMap;
/**
 * An algorithm that computes a solution of the motion planning problem. <br>
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, dec 2020)
 *
 */
public class MyBestAlgorithm extends MotionAlgorithm {
	/** An input instance of the motion planning problem */
	public Instance input;
	
	/** The solution computed by the algorithm */
	public Solution solution;
	
	/** Current locations of robots */
	Coordinates current;
	
	HashMap<Coord, Boolean> currentRobotPos;
	
	public MyBestAlgorithm(Instance input) {
		this.input=input;
		this.solution=new Solution(input.name); // create an empty solution (no steps at the beginning)
		this.current=new Coordinates(this.input.starts.getPositions()); // initialize the current locations with the starting input locations
		this.currentRobotPos = new HashMap<Coord, Boolean>();
		for (int i = 0; i < input.starts.size(); i++) {
			Coord point = new Coord(input.starts.getX(i), input.starts.getY(i));
			currentRobotPos.put(point, true);
		}
	}
	
	/**
	 * Return the current solution: it assumes that the solution has been computed
	 */
	public Solution getSolution() {
		return this.solution;
	}
	
	public void test() {
		int size = current.size();
		Trajectories traj = new Trajectories(input, current);
		int num = 20;
		byte[] directions = new byte[size];
		for (int i = 0; i < num; i++) {
			for (int j = 0; j < size; j++) {
				directions[j] = traj.moveRandomly(j, true, false);

			}
			solution.addStep(directions);
			directions = new byte[size];
		}
		/*int num = 5;
		byte[] directions = new byte[size];
		for (int i = 0; i < num; i++) {
			directions = traj.moveAllRobotsRandomly(true, false);
			solution.addStep(directions);
			Trajectories.printArray(directions, size);
			directions = new byte[size];
		}
		
		System.out.println(traj);
		
		// Now we place them on target
		run_2();
				
		traj = new Trajectories(input, current);
		System.out.println(traj);

		// Now move some robots randomly
		int robotMove = 3;
		
		directions = new byte[size];
		for (int i = 0; i < robotMove; i++) {
			directions[i] = traj.moveRandomly(i, true, false);
		}
		for (int i = robotMove; i < size; i++) {
			directions[i] = Solution.FIXED;
		}
		solution.addStep(directions);
		Trajectories.printArray(directions, size);
		directions = new byte[size];
		// Now move robots on target
		directions = traj.moveRobotsOnTarget();
		solution.addStep(directions);
		Trajectories.printArray(directions, size);
		
		// print final trajectories status
		System.out.println(traj);
		
		// print solution
		for(byte[] step : solution.getSteps())
			Trajectories.printArray(step, size);
			*/
		
	}
	
	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	// Here, we follow optimal roadMap calculated on first step, taking into account robots positions.
	// If a robot cannot get to target, it waits and regenerates roadMap taking into account actual position
	// If a robot cannot move, it does the same
	public void run_1() {
		// TO BE WRITTEN
		int size = current.size();
		//byte[] bytes = new byte[size];
		
		Trajectories traj = new Trajectories(input, current);
		// Remember it is possible that some robot stay without roadMap because of roadblocks.
		traj.initRoadMaps(Trajectories.ROBOT_IS_OBST);
		byte[] directions;
		while(!traj.areAllRobotsFinished()) {
			directions = new byte[size];
			for (int i = 0; i < size; i++) {
				if (traj.noValidRoadMap(i)) {
					// try to see i new route available, and wait another tour
					traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
					directions[i] = Solution.FIXED;
				}
				else {
					byte dir = traj.getNextDir(i);
					if(traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST)) {
						directions[i] = dir;
					}
					else {
						directions[i] = Solution.FIXED;
						traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
					}
				}
			}
			solution.addStep(directions);
		}
		System.out.println("Solution computed");
	}
	
	// Here, we follow optimal roadMap calculated on first step.
	// If a robot is blocked on its roadMap, it recalculates optimal roadMap taking into account other robots
	// and does one step in that direction; then it regenerates a roadMap forgetting about other robots.
	// If it cannot get to its target whatsoever, it waits.
	// 26, 124
	public void run_2() {
		// TO BE WRITTEN
		int size = current.size();
		//byte[] bytes = new byte[size];
		
		Trajectories traj = new Trajectories(input, current);
		// Creating RoadMaps without taking into account other robots
		traj.initRoadMaps(Trajectories.ROBOT_IS_NOT_OBST);
		
		byte[] directions;
		while(!traj.areAllRobotsFinished()) {
			directions = new byte[size];
			for (int i = 0; i < size; i++) {
				// if no valid roadMap available
				if (traj.noValidRoadMap(i)) {
					// tries to recalculate roadMap without taking account other robots.
					// if that fails, there is no way for it to get there whatsoever.
					// so it'll wait.
					traj.updateRoadMap(i, Trajectories.ROBOT_IS_NOT_OBST);
					if (traj.noValidRoadMap(i)) {
						directions[i] = Solution.FIXED;
					}
				}
				// Now there is a valid roadMap.
				byte dir = traj.getNextDir(i);
				if (traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST)) {
					// ok, movement possible
					directions[i] = dir;
				}
				else {
					// Movement impossible. Recalculating route taking into account other robots
					traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
					if (traj.noValidRoadMap(i)) {
						directions[i] = Solution.FIXED;
					}
					else {
						dir = traj.getNextDir(i);
						traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST);
						directions[i] = dir;
					}
				}
			}
			
			solution.addStep(directions);
		}
		System.out.println("Solution computed");
	}
	
	// Here, we follow optimal roadMap(true) at each step.
	// If a robot is blocked on its roadMap, it waits
	// If it cannot get to its target whatsoever, it waits.
	// 24, 114
	public void run_3() {
		// TO BE WRITTEN
		int size = current.size();
		//byte[] bytes = new byte[size];
		
		Trajectories traj = new Trajectories(input, current);
		// Creating RoadMaps without taking into account other robots
				traj.initRoadMaps(Trajectories.ROBOT_IS_OBST);
				
				byte[] directions;
				while(!traj.areAllRobotsFinished()) {
					directions = new byte[size];
					for (int i = 0; i < size; i++) {
						// recalculates
						traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
						// if no valid roadMap available
						if (traj.noValidRoadMap(i)) {
							// wait
								directions[i] = Solution.FIXED;
						}
						else {
							// Now there is a valid roadMap.
							byte dir = traj.getNextDir(i);
							traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST);
							directions[i] = dir;
						}
					}
					
					solution.addStep(directions);
				}
				System.out.println("Solution computed");
			}
	
	// Here, we calculate roadMap(true) at each turn.
	// if it there is no valid roadMap(true), it waits one turn, 
	// and then we ask all robots on target to move randomly one step to let it breathe again
	// 23, 179
	public void run_4() {
		// TO BE WRITTEN
		int size = current.size();
		
		Trajectories traj = new Trajectories(input, current);
		// Creating RoadMaps taking into account other robots
		traj.initRoadMaps(Trajectories.ROBOT_IS_OBST);
		
		byte[] directions;
		
		int[] waited = new int[size];
		for (int i = 0; i < size; i++)
			waited[i] = 0;
		
		while(!traj.areAllRobotsFinished()) {
			directions = new byte[size];
			for (int i = 0; i < size; i++) {
				traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
				// if no valid roadMap available
				if (traj.noValidRoadMap(i)) {
					// oupsi
					if (waited[i] == 0) {
						directions[i] = Solution.FIXED;
						waited[i]++;
					}
					else {
						byte[] newDir = traj.moveRobotsOnTarget();
						zeroPad(directions, i, size);
						solution.addStep(directions);
						solution.addStep(newDir);
						continue;						
					}
				}
				else {
					waited[i] = 0;
					directions[i] = traj.getNextDir(i);
					traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST);
				}
			}
			
			solution.addStep(directions);
		}
		System.out.println("Solution computed");
	}
	
		// Here, we calculate roadMap(false) initially.
		// If there is no movement possible, wait. Then, try roadMap(true). On failure, wait one turn, 
		// and on failure again we ask all robots on target to move randomly one step to let graph breathe again
		// if that fails too, on next turn we ask everybody to move randomly, and try to roadMap(true) one step.
		// Then back to roadMap(false).
		public void run_5() {
			// TO BE WRITTEN
			int size = current.size();
			
			Trajectories traj = new Trajectories(input, current);
			// Creating RoadMaps without taking into account other robots
			traj.initRoadMaps(Trajectories.ROBOT_IS_NOT_OBST);
			
			byte[] directions;
			
			int[] waited = new int[size];
			for (int i = 0; i < size; i++)
				waited[i] = 0;
			
			int[] strategySwitch = new int[4];
			strategySwitch[0] = 3; // No movement
			strategySwitch[1] =  4; // move randomly
			strategySwitch[2] =  6; // update roadMap with robots taken into account. Do not update later
			strategySwitch[3] =  7; // move robots on target
			 // else just move everybody 
			
			int span = 0;
			while(!traj.areAllRobotsFinished() && span < 500) {
				span++;
				directions = new byte[size];
				boolean interruptAndRestart = false;
				for (int i = 0; i < size; i++) {
					if (traj.isFinished(i)) {
						directions[i] = Solution.FIXED;
						continue;
					}
					
					// If no movement possible.
					//System.out.println(String.format("[" + span + "] " + i + " : " + waited[i]));
					if ((waited[i] >= strategySwitch[0] && waited[i] < strategySwitch[2])) {
						//System.out.println(i + " : Random");
						byte dir = traj.moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_OBST);
						directions[i] = dir;
						waited[i]++;
						continue;
					}
					
					byte dir = Solution.FIXED;
					boolean enterStrategy = false;
					if (traj.noValidRoadMap(i)) {
						enterStrategy = true;
					}
					else {
						dir = traj.getNextDir(i);
						if (!traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST))
							enterStrategy = true;
					}
					
					if(enterStrategy) {
						// oupsi
						// now different strategies
						if (waited[i] < strategySwitch[0]) {
							//System.out.println(i + " : FIXED");
							directions[i] = Solution.FIXED;
							waited[i]++;
						}
						else if(waited[i] >= strategySwitch[0] && waited[i] < strategySwitch[1]) {
							//System.out.println("Random");
							dir = traj.moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_OBST);
							directions[i] = dir;
							waited[i]++;
						}
						else if (waited[i] >= strategySwitch[1] && waited[i] < strategySwitch[2]) {
							//System.out.println(i + " : ROADmAP++");
							traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
							if (traj.noValidRoadMap(i)) {
								System.out.println("FUCK");
								directions[i] = Solution.FIXED;
								waited[i]++;
							}
							else {
								directions[i] = traj.getNextDir(i);
								traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST);
								waited[i] = 0;
							}
						}
						else if (waited[i] >= strategySwitch[2] && waited[i] < strategySwitch[3]) {
							//System.out.println(i + " : MoveRobotsTarget");
							//System.out.println("Moving robots on target on span number " + span);
							//System.out.println(traj);
							byte[] newDir = traj.moveRobotsOnTarget();
							//System.out.println(traj);
							// adding intermediate mov
							zeroPad(directions, i, size);
							solution.addStep(directions);
							solution.addStep(newDir);
							// recalculate route
							traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
							waited[i]++;
							
							// back to beginning of while loop
							/*i = size;
							continue;*/
							interruptAndRestart = true;
							break;
						}
						else if (waited[i] >= strategySwitch[3]) {
							//System.out.println(i + " : Move all");
							//System.out.println("Moving robots on target on span number " + span);
							//System.out.println(traj);
							byte[] newDir = traj.moveAllRobotsRandomly(Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_NOT_OBST);
							//System.out.println(traj);
							zeroPad(directions, i, size);
							solution.addStep(directions);
							solution.addStep(newDir);
							traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
							waited[i]++;
							// back to beginning of while loop
							/*i = size;
							continue;*/
							interruptAndRestart = true;
							break;
						}
						
						// And we DO NOT update roadMap so that it is roadMap(false)
						//traj.updateRoadMap(i, Trajectories.ROBOT_IS_NOT_OBST);						
					}
					else {
						waited[i] = 0;
						/*System.out.println(String.format("Current (" + i + ") : " + 
													current.getCoord(i) + " - Target : " 
													+ input.targets.getCoord(i) + " - " + traj.getRoadMap(i)));*/
						directions[i] = dir;
					}
				}
				if (!interruptAndRestart) {
					//System.out.println("Step " + span);
					//Trajectories.printArray(directions, size);
					//Trajectories.printArray(waited, size);
					//System.out.println(traj);
					solution.addStep(directions);
				}
			}
			System.out.println("Solution computed");
	}

		// Here, we calculate roadMap(true) initially. If blocked, it waits. On next step, recalculate.
		// it waits one turn, 
		// and then we ask all robots to move randomly one step to let it breathe again
		public void run() {
			// TO BE WRITTEN
						int size = current.size();
						
						Trajectories traj = new Trajectories(input, current);
						// Creating RoadMaps without taking into account other robots
						traj.initRoadMaps(Trajectories.ROBOT_IS_OBST);
						
						byte[] directions;
						
						int[] waited = new int[size];
						for (int i = 0; i < size; i++)
							waited[i] = 0;
						
						int[] strategySwitch = new int[4];
						strategySwitch[0] = 2; // No movement
						strategySwitch[1] =  2; // move randomly
						strategySwitch[2] =  5; // update roadMap with robots taken into account. Do not update later
						strategySwitch[3] =  5; // move robots on target
						 // else just move everybody 
						
						int span = 0;
						while(!traj.areAllRobotsFinished() && span < 500) {
							span++;
							directions = new byte[size];
							boolean interruptAndRestart = false;
							for (int i = 0; i < size; i++) {
								if (traj.isFinished(i)) {
									directions[i] = Solution.FIXED;
									continue;
								}
								
								// If no movement possible.
								//System.out.println(String.format("[" + span + "] " + i + " : " + waited[i]));
								if ((waited[i] >= strategySwitch[0] && waited[i] < strategySwitch[2])) {
									//System.out.println(i + " : Random");
									byte dir = traj.moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_OBST);
									directions[i] = dir;
									waited[i]++;
									continue;
								}
								
								byte dir = Solution.FIXED;
								boolean enterStrategy = false;
								if (traj.noValidRoadMap(i)) {
									enterStrategy = true;
								}
								else {
									dir = traj.getNextDir(i);
									if (!traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST))
										enterStrategy = true;
								}
								
								if(enterStrategy) {
									// oupsi
									// now different strategies
									if (waited[i] < strategySwitch[0]) {
										//System.out.println(i + " : FIXED");
										directions[i] = Solution.FIXED;
										waited[i]++;
									}
									else if(waited[i] >= strategySwitch[0] && waited[i] < strategySwitch[1]) {
										//System.out.println("Random");
										dir = traj.moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_OBST);
										directions[i] = dir;
										waited[i]++;
									}
									else if (waited[i] >= strategySwitch[1] && waited[i] < strategySwitch[2]) {
										//System.out.println(i + " : ROADmAP++");
										traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
										if (traj.noValidRoadMap(i)) {
											System.out.println("FUCK");
											directions[i] = Solution.FIXED;
											waited[i]++;
										}
										else {
											directions[i] = traj.getNextDir(i);
											traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST);
											waited[i] = 0;
										}
									}
									else if (waited[i] >= strategySwitch[2] && waited[i] < strategySwitch[3]) {
										//System.out.println(i + " : MoveRobotsTarget");
										//System.out.println("Moving robots on target on span number " + span);
										//System.out.println(traj);
										byte[] newDir = traj.moveRobotsOnTarget();
										//System.out.println(traj);
										// adding intermediate mov
										zeroPad(directions, i, size);
										solution.addStep(directions);
										solution.addStep(newDir);
										// recalculate route
										traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
										waited[i]++;
										
										// back to beginning of while loop
										/*i = size;
										continue;*/
										interruptAndRestart = true;
										break;
									}
									else if (waited[i] >= strategySwitch[3]) {
										//System.out.println(i + " : Move all");
										//System.out.println("Moving robots on target on span number " + span);
										//System.out.println(traj);
										byte[] newDir = traj.moveAllRobotsRandomly(Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_NOT_OBST);
										//System.out.println(traj);
										zeroPad(directions, i, size);
										solution.addStep(directions);
										solution.addStep(newDir);
										traj.updateRoadMap(i, Trajectories.ROBOT_IS_OBST);
										waited[i]++;
										// back to beginning of while loop
										/*i = size;
										continue;*/
										interruptAndRestart = true;
										break;
									}
									
									// And we DO NOT update roadMap so that it is roadMap(false)
									//traj.updateRoadMap(i, Trajectories.ROBOT_IS_NOT_OBST);						
								}
								else {
									waited[i] = 0;
									/*System.out.println(String.format("Current (" + i + ") : " + 
																current.getCoord(i) + " - Target : " 
																+ input.targets.getCoord(i) + " - " + traj.getRoadMap(i)));*/
									directions[i] = dir;
								}
							}
							if (!interruptAndRestart) {
								//System.out.println("Step " + span);
								//Trajectories.printArray(directions, size);
								//Trajectories.printArray(waited, size);
								//System.out.println(traj);
								solution.addStep(directions);
							}
						}
						System.out.println("Solution computed");
		}



		
	// NON NON NON !!! Si l'on demande à bouger tous les robots par exemple, on finit de remplir directions avec des Solution.FIXED,
	// puis on envoie l'instruction suivante. Il nous faut donc plutot une fonction qui zero pad
	private void zeroPad(byte[] directions, int i, int size) {
		for (int j = i; j < size; j++)
			directions[j] = Solution.FIXED;
	}
	/*private void addNewStepInMiddle(Solution solution, int size, byte[] directions, byte[] newDirections, int i, boolean included) {
		// also adding intermdeaite mov. Do not forget to include the i-th robot which has also moved !
		int k  = included ? i : i + 1;
		for (int j = k; j < size; j++) {
			directions[j] = newDirections[j];
		}
		solution.addStep(directions);
		directions = new byte[size];
		for (int j = 0; j < i; j++) {
			directions[j] = newDirections[j];
		}
	}*/
		
	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	
	public void run_bis() {
		// TO BE WRITTEN
		int size = current.size();
		
		Trajectories traj = new Trajectories(input, current);
		traj.initRoadMaps(Trajectories.ROBOT_IS_OBST);
				
		// Array containing next directions for each step
		byte[] directions;	
		
		// Beginning algorithm
		while(!traj.areAllRobotsFinished()) {
			// Initializing array to be passed to solution
			directions = new byte[size];
			
			// Now working on each robot
			for (int i = 0; i < size; i++) {
				// If robot has already reached destination
				if(traj.isFinished(i)) {
					// It must not move
					directions[i] = Solution.FIXED;
					continue;
				}
				
				// get next planned direction on roadMap
				System.out.print(String.format("robot " + i + " : " + current.getCoord(i).toString()));
				System.out.print(" - > ");
				byte dir = traj.getNextDir(i);
				// If a move can and has been made
				if (traj.followRoadMap(i, Trajectories.ROBOT_IS_OBST)) {
					directions[i] = dir;
				}
				else {
					// robot cannot move optimaly.
					// trying to move randomly
					directions[i] = traj.moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_OBST);
				}
				
				System.out.print(current.getCoord(i));
				
				// We check if robot has reached destination
				if (traj.isFinished(i)) {
					System.out.print(" | END");
				}
				
				System.out.println();

			}
			
			// This step is finished, we add it to solution
			solution.addStep(directions);
			
			// And we get one step further to the truth
		}
		
		System.out.println("Solution computed");
	}
	
	/**
	 * Add a new motion step to the current solution
	 */
	
	public void computeOneStep() {
	}
	
	

}
