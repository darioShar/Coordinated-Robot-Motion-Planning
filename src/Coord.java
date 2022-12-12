
public class Coord {
	public int x;
	public int y;
	
	public Coord(int a, int b) {
		x = a;
		y = b;
	}
	
	public Coord nextPos(byte dir) {
		switch(dir) {
		case(Solution.FIXED) :
			return new Coord(x, y);
		case(Solution.N) :
			return new Coord(x, y + 1);
		case(Solution.S) :
			return new Coord(x, y - 1);
		case(Solution.E) :
			return new Coord(x + 1, y);
		case(Solution.W) :
			return new Coord(x - 1, y);
		default :
			return new Coord(x, y);
		}
	}
	
	@Override
	public String toString() {
		return String.format(x + ", " + y);
	}
	
	@Override
	public boolean equals(Object o) {
		if (x == ((Coord)o).x && y == ((Coord)o).y) return true;
		
		return false;
	}
	
	@Override
	public int hashCode() {
	    return x * 31 + y;
	}
}
