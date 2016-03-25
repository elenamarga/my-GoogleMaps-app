package roadgraph;
import geography.GeographicPoint;

// A class which represents an edge (i.e. a road).
public class Edge {
	private Node start;
	private Node end;
	private String roadName;
	private String roadType;
	private double length;

	static final double DEFAULT_LENGTH = 0.01;
	
	// Create a new Edge.
	public Edge(Node from, Node to, String roadName, String roadType, 
			double length) {
		start = from;
		end = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	// Get the Node for the end point.
	public Node getEndNode() {
		return end;
	}
	
	// Get the location of the end point.
	public GeographicPoint getEndPoint() {
		return getEndNode().getLocation();
	}
	
	// Get the Node for the start point
	public Node getStartNode() {
		return start;
	}
	
	// Get the location of the start point.
	public GeographicPoint getStartPoint() {
		return getStartNode().getLocation();
	}
	
	// Get the name of the road.
	public String getRoadName() {
		return roadName;
	}
	
	// Get the type of the road.
	public String getRoadType() {
		return roadType;
	}
	
	// Get the length of the road.
	public double getLength() {
		return length;
	}
	
	// Given one node in an edge, get the other node.
	public Node getOtherNode(Node node) {
		if (node.equals(start)) {
			return end;
		} else if (node.equals(end)) {
			return start;
		}
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
}
