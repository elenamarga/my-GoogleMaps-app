package roadgraph;
import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

// A class which represents a graph node (i.e. an intersection).
public class Node implements Comparable {
	private GeographicPoint location;
	private HashSet<Edge> edges;
	
	private double actualScore;
	private double score;

	
	// Create a new Node, with the given location, no edges and no neighbors.
	public Node(GeographicPoint loc) {
		location = loc;
		edges = new HashSet<Edge>();
		actualScore = 0.0;
		score = 0.0;
	}
	
	// Add the given edge to the list of edges and updates the list of neighbors,
	// by adding the node "neighbor" (which corresponds to the end of 
	// the added edge). 
	public void addEdge(Edge newEdge) {
		edges.add(newEdge);
	}
	
	// Get the location of the node.
	public GeographicPoint getLocation() {
		return location;
	}
	
	// Get the edges of the node.
	public Set<Edge> getEdges() {
		return edges;
	}
	
	// Get the neighbors of the node.
	public Set<Node> getNeighbors() {
		Set<Node> neighbors = new HashSet<Node>();
		for (Edge edge : getEdges()) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	// Get the score from the Node to the start Node.
	public double getActualScore() {
		return actualScore;
	}
	
	// Get the estimated score from the Node to the goal Node.
	public double getScore() {
		return score;
	}
	
	// Set the score from the Node to the start Node.
	public void setActualScore(double score) {
		actualScore = score;
	}
	
	// Set the estimated score from the Node to the goal Node.
	public void setScore(double score) {
		this.score = score;
	}

	@Override
	public int compareTo(Object o) {
		Node node = (Node)o;
		return ((Double)this.getScore()).compareTo(
				(Double)node.getScore());
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 */
	public boolean equals(Object o)
	{
		if (!(o instanceof Node) || (o == null)) {
			return false;
		}
		Node node = (Node)o;
		return node.location.equals(this.location);
	}
	
	public double getDistanceTo(Node goalNode) {
		return getLocation().distance(goalNode.getLocation());
	}
	
	public double getTimeTo(Node goalNode) {	
		return getDistanceTo(goalNode) / MapGraph.SPEED_MOTORWAY;
	}
}
