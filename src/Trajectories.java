import java.util.ArrayList;
import java.util.Collections;

import java.lang.*;
import java.util.HashMap;

/** 
 * Cette classe permet d'abstraitiser l'idée de feuille de route. Elle prend en entrée l'instance d'entrée
 * input et la position actuelle des robots current qu'elle va modifier. Elle met en place un simple dijkstra
 * prenant en compte la position des obstacles et la position des robots (ou non, au choix) pour générer une 
 * feuille de route personnalisée pour chaque robot, de sa position à la position de sa target.
 * Le roadMaps est intialisé avec des feuilles de routes null. On utlise la méthode initRoadMaps
 * Pour la remplir avec les premières feuilles de route.
 * Ensuite, on peut demander àobtenir la prohaine direction planifiée (getNextPlannedDir), 
 * à faire bouger un robot selon la feuille de route (followRoadMap). Pour le moment, on empeche des mvt quelconques :
 * move reste private.
 *
 */
public class Trajectories {
	public final static boolean ROBOT_IS_NOT_OBST = false, ROBOT_IS_OBST = true; 
	
	boolean[] finished;
	int finishedRobots;
	int size;
	ArrayList<ArrayList<Byte>> roadMaps;
	// Keeping track of robot position in its actual roadmap
	byte[] roadPos;
	
	Coordinates current;
	HashMap<Coord, Boolean> currentRobotPos;
	Instance input;
	HashMap<Coord, Boolean> hashObstacles;
	
	// Array for shuffling for later random move
	ArrayList<Byte> randomMovement = new ArrayList<Byte>();
	
	public Trajectories(Instance input, Coordinates current) {
		this.current = current;
		this.input = input;
		
		this.finishedRobots = 0;
		this.size = current.size();
		this.finished = new boolean[size];
		// How many already finished robots. Normally zero
		for (int i = 0; i < this.size; i++) {
			if (current.isEqualIndex(input.targets, i)) {
				finished[i] = true;
				finishedRobots++;
			}
		}
		
		this.roadMaps = new ArrayList<ArrayList<Byte>>();
		for (int i = 0; i < this.size; i++)
			roadMaps.add(null);
		
		this.roadPos = new byte[size];
		for (int i = 0; i < this.size; i++) {
			roadPos[i] = 0;
		}
		
		// And calculating hashes
		this.hashObstacles = new HashMap<Coord, Boolean>();
    	this.currentRobotPos = new HashMap<Coord, Boolean>();
    	for (int i = 0; i < this.size; i++) {
    		this.currentRobotPos.put(current.getCoord(i), true); 
		}
    	for (int i = 0; i < this.input.obstacles.size(); i++) {
    		this.hashObstacles.put(this.input.obstacles.getCoord(i), true); 
		}
    	
    	this.randomMovement.add(Solution.N);
    	this.randomMovement.add(Solution.S);
    	this.randomMovement.add(Solution.E);
    	this.randomMovement.add(Solution.W);
    	
	}
	
	@Override
	public String toString() {
		String s = "";
		s += "Total number of robots : " + String.valueOf(size) + ". Finished : \n";
		for (int i = 0; i < size; i++) {
			if (finished[i])
				s += "1 : ";
			else
				s += "0 : ";
		}
		s += "total finished = " + finishedRobots + "\n";
		s += "Status fo robots : \n";
		for (int i = 0; i < size; i++) {
			s += String.valueOf(i) + " : ";
			s += "\t(start) " + input.starts.getCoord(i) + " ; (current) " + current.getCoord(i) + " ; (target) " + input.targets.getCoord(i) + "\n";
			s += "\t RoadMap (roadPos=" + roadPos[i] + "): " + roadMaps.get(i) + "\n";
		}
		
		return s;
	}
	
	public boolean isFinished(int i) {
		return finished[i];
	}
	
	public boolean areAllRobotsFinished() {
		return finishedRobots == size;
	}
	
	public boolean noValidRoadMap(int i) {
		return !finished[i] && roadMaps.get(i) == null;
	}
    
    public void updateRoadMap(int i, boolean isRobotObst) {
    	roadMaps.set(i, updateTrajectory(i, isRobotObst));
		roadPos[i] = 0;
    }
	
	public void initRoadMaps(boolean isRobotObst) {
		for (int i = 0; i < size; i++) {
			updateRoadMap(i, isRobotObst);
		}
	}
	
	public byte getNextDir(int i) {
		if (finished[i])
			return Solution.FIXED;
		return roadMaps.get(i).get(roadPos[i]);
	}
	
	public ArrayList<Byte> getRoadMap(int i) {
		return roadMaps.get(i);
	}
	
	public boolean followRoadMap(int i, boolean isRobotObst) {
		if (finished[i])
			return true;
		
		byte dir = roadMaps.get(i).get(roadPos[i]);
		if (move(i, dir, isRobotObst)) {
			roadPos[i]++;
			return true;
		}
		return false;
		
	}
	
	private boolean move(int i, byte dir, boolean isRobotObst) {
		if (dir == Solution.FIXED)
			return true;
		
		Coord robot = current.getCoord(i);
		Coord next = robot.nextPos(dir);
		if (hashObstacles.get(next) != null)
			return false;
		
		if (!isRobotObst || this.currentRobotPos.get(next) == null) {
			// updating current
			current.setX(i, next.x);
			current.setY(i, next.y);
			
			// updating hashmap
			currentRobotPos.remove(robot);
			currentRobotPos.put(next, true);
			
			// See if finished or not
			if (current.getCoord(i).equals(input.targets.getCoord(i))) {
				finished[i] = true;
				finishedRobots++;
			}
			else if (finished[i]) {
				finished[i] = false;
				finishedRobots--;
			}
			return true;
		}
		return false;
	}
	
	/* Tries to move randomly and regenerate roadMap. On failure, do nothing. */
	public byte moveRandomly(int i, boolean isRobotObstOnMove, boolean isRobotObstOnRoadMap) {
		Collections.shuffle(randomMovement);
		byte finalDir = Solution.FIXED;
		for (byte dir : randomMovement) {
			// if another movement is possible
			if (move(i, dir, isRobotObstOnMove)) {
				
				// update roadmap and reinitiliaze roadPos
				updateRoadMap(i, isRobotObstOnRoadMap);
				
				// If the robot is blocked, i.e there is no satisfying trajectory
				// (remark : we should then update the roadmap of another robot to "deblock" the way for this robot)
				if (getRoadMap(i) == null) {
					// for the moment, end of work.
					//robotHasFinished(i);
					System.out.print(" | PROBLME IN RANDOM MOVEMENT |"); 
				}
				finalDir = dir;
				break;
			}
		}
		return finalDir;
	}
	
	// randomly moves all robots on target and update their respective roadMaps
	public byte[] moveRobotsOnTarget() {
		byte[] arr = new byte[size];
		for (int i = 0; i < size; i++) {
			byte dir = Solution.FIXED;
			if (finished[i]) {
				dir = moveRandomly(i, Trajectories.ROBOT_IS_OBST, Trajectories.ROBOT_IS_NOT_OBST);
				//System.out.print(String.format("Trying to move " + i + " in direction " + dir + "..."));
			}
			arr[i] = dir;
		}
		return arr;
	}
	
	// randomly moves all robots and update their respective roadMaps
	public byte[] moveAllRobotsRandomly(boolean isRobotObstOnMove, boolean isRobotObstOnRoadMap) {
		byte[] arr = new byte[size];
		for (int i = 0; i < size; i++) {
			byte dir = moveRandomly(i, isRobotObstOnMove, isRobotObstOnRoadMap);
			arr[i] = dir;
		}
		return arr;
	}
	
	// Renvoie un tableau indiquant le trajet optimal. Renvoie null en cas d'échec.
	// Attention : mon programme a buggé pendant longtemps parce que j'ai considéré target libre
	// d'autre robot...
	private ArrayList<Byte> updateTrajectory(int idx, boolean isRobotObst) 
		{
			// First, reinitialize roadPos
			roadPos[idx] = 0;
			// Pour le moment, s'il y a un robot sur target, on renvoie null, on ne sait pas faire.
			// ATTENTION !! il se peut que target ne soit qu'à un seul pas, mais qu'il y ait un robot
			// déjà dessus...
			if (isRobotObst && isOnRobot(input.targets.getCoord(idx))) {
				System.out.println(String.format("Robot " + idx + " cannot get to target : blocked by another robot"));
				return null;
			}
		
		
			//System.out.println("updating trajectory...");
			
			// Désormais on récupère la délimitation des trajectoires empruntables par le robot. 
			// Bien que l'on ne fasse pas attention à la présence mutuelle des robots, il se peut que durant le déroulement 
			// de l'algorithme les robots soient contraints de sortir de la délimitation rectangulaire initiale.
			// Par conséquent, on recalcule leurs bounding box
			Instance new_input = new Instance("", current, input.targets, input.obstacles);
			new_input.getBoundingBox();
			
			
			// Remarque : on peut imaginer que l'on a un obstacle placé sur le bord. Il serait dommage de ne pas pouvoir
			// le contourner ! Bref ce n'est pas le seul cas où considérer un rectangle dont largeur et longueur sont augmentés de deux 
			// (en haut et en bas) rend l'algorithme plus efficace. C'est ce que nous allons faire.
			/* 1 1 1 1
			 * 1 0 2 1
			 * 1 2 0 1
			 * 1 1 1 1
			 * devient 
			 * 1 1 1 1 1 1
			 * 1 0 0 0 0 1
			 * 1 0 0 2 0 1
			 * 1 0 2 0 0 1
			 * 1 0 0 0 0 1
			 * 1 1 1 1 1 1
			 */
			
			// get start positions et redimensionnement
			Coord start = new_input.starts.getCoord(idx);
			start.x -= new_input.xmin;
			start.y -= new_input.ymin;
			// et on augmente la taille du rectangle
			start.x += 1;
			start.y += 1;
			
			// idem on redimensionne target
			Coord target = new_input.targets.getCoord(idx);
			target.x -= new_input.xmin;
			target.y -= new_input.ymin;
			// et on augmente la taille du rectangle
			target.x += 1;
			target.y += 1;
					
			int width = new_input.xmax - new_input.xmin + 1;
			int height = new_input.ymax - new_input.ymin + 1;
			
			// on augmente la taille du rectangle
			width += 2;
			height += 2;
			
			// Pour chaque case, un tableau est crée pour déterminer le chemin optimal de start[i] à target[i]. Idem pour nombre de pas.
			// On pourrait en avoir besoin par la suite.
			int[][] optimalSteps = new int[height][width];
			byte[][] optimalPaths = new byte[height][width];
			
			
			// initialisation des deux tableaux
			for (int i = 0; i < height; i++) {
				for (int j = 0; j < width; j++) {
					optimalSteps[i][j] = Integer.MAX_VALUE;
					optimalPaths[i][j] = -1; // -1 signifie donc que l'on n'a pas atteint cette case là.
				}
			}
			
			// init au point de départ
			optimalSteps[start.y][start.x] = 0;
			optimalPaths[start.y][start.x] = Solution.FIXED;
			
			// buffer qui contient le bfs
			ArrayList<Coord> buffer = new ArrayList<Coord>();
			buffer.add(start);
			
			// tenir un pointeur vers la position actuelle dans le bfs
			int actualPos = 0;
			
			// Visite des enfants de la position actuelle et extension
			while(true) {
				// S'il n'y a plus d'enfants à aller voir : c'est un échec. On renvoie null
				if (actualPos == buffer.size()) {
					System.out.println(String.format("Robot " + idx + " cannot get to target : path is blocked by obstacles or robots"));
					return null;
				}
				
				// Récupérer le père depuis lequel on fait le bfs
				Coord root = buffer.get(actualPos);
				
				// Récupération des enfants
				Coord e = new Coord(root.x + 1, root.y);
				Coord n = new Coord(root.x, root.y + 1);
				Coord w = new Coord(root.x - 1, root.y);
				Coord s = new Coord(root.x, root.y - 1);
				
				// ajout des enfants et remplissage des tableaux de chemin et de nombre de pas.
				addSon(buffer, e, root, optimalSteps, optimalPaths, width, height, Solution.E, new_input, isRobotObst);
				addSon(buffer, n, root, optimalSteps, optimalPaths, width, height, Solution.N, new_input, isRobotObst);
				addSon(buffer, w, root, optimalSteps, optimalPaths, width, height, Solution.W, new_input, isRobotObst);
				addSon(buffer, s, root, optimalSteps, optimalPaths, width, height, Solution.S, new_input, isRobotObst);
				
				// Vérification : le target est il atteint ?
				if (target.equals(e) || target.equals(n) || target.equals(w) || target.equals(s)) {
					//System.out.println(String.format("in, with i = " + idx + ", start = " + start + ", target = " + target));
					
					// tableau qui contiendra les steps nécessaires pour atteindre target, rempli à l'envers
					// (on part de target pour revenir à start).
					ArrayList<Byte> reversedPath = new ArrayList<Byte>();
					
					// on remonte
					while (!target.equals(start)) {
						byte dir = optimalPaths[target.y][target.x];
						//System.out.print(dir);
						//System.out.print(" : " + target.toString() + " ->");
						// En fonction de la direction à partir de laquelle on a atteint la case lors du bfs
						switch(dir) {
						case Solution.E :
							target.x -= 1;
							break;
						case Solution.N :
							target.y -= 1;
							break;
						case Solution.W : 
							target.x += 1;
							break;
						case Solution.S :
							target.y += 1;
							break;
						default :
							// en cas de bug
							System.out.println("Problem !");
							printGrid(optimalSteps, height, width);
							printGrid(optimalPaths, height, width);
							current.print();
							return reversedPath;
						}
						//System.out.println(target);
						reversedPath.add(dir);
					}
					//System.out.println("out");
					// On renvoie la solution
					Collections.reverse(reversedPath);
					return reversedPath;
				}
				
				// On avance de 1 dans le bfs
				actualPos++;
			}			
		}
		
		// Ajout des enfants.
		private void addSon(ArrayList<Coord> buffer, Coord son, Coord root, int[][] steps, byte[][] paths,
				int width, int height, byte dir, Instance new_input, boolean isRobotObst) 
		{
			// On vérifie que le point n'est pas sorti du rectangle de travail.
			// On fera attention à retraduire les coordonnées des points surlesquels on travaille
			// (avec input.xmin et input.ymin) et bien sûr on n'oubliera pas que l'on a aussi
			// "augmenté" notre rectangle de départ !
			Coord point = new Coord( son.x + new_input.xmin - 1, son.y + new_input.ymin - 1);
			if (son.x >= 0 && son.x < width && son.y >= 0 && son.y < height
				&& paths[son.y][son.x] == -1 
				&& !isOnObstacle(point) && (!isRobotObst || !isOnRobot(point)))
			{
				buffer.add(son);
				steps[son.y][son.x] = steps[root.y][root.x] + 1;
				paths[son.y][son.x]= dir;
			}
		}
		
		public boolean isOnRobot(Coord point) {
			return (currentRobotPos.get(point) != null);
		}
	    public boolean isOnObstacle(Coord point) {
	    	return (hashObstacles.get(point) != null);
	    }
				
		public static void printGrid(byte[][] grid, int n, int m) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					System.out.print(grid[i][j]);
					System.out.print(", ");
				}
				System.out.println();
			}
				
		}
		public static void printGrid(int[][] grid, int n, int m) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					if (grid[i][j] == Integer.MAX_VALUE) {
						System.out.print("n");
					}
					else 
						System.out.print(grid[i][j]);
					System.out.print(", ");
				}
				System.out.println();
			}
				
		}
		
		public static void printArray(byte[] arr, int size) {
			for (int i = 0; i < size; i++) {
				System.out.print(arr[i]);
				System.out.print(", ");
			}
			System.out.println();
		}
		
		public static void printArray(int[] arr, int size) {
			for (int i = 0; i < size; i++) {
				System.out.print(arr[i]);
				System.out.print(", ");
			}
			System.out.println();
		}
	
}
