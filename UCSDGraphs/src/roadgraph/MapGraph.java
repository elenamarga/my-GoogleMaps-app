/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Mapping from GeographicPoint to Node
	private HashMap<GeographicPoint, Node> nodes;
	private HashSet<Edge> edges;
	
	final static double SPEED_MOTORWAY = 120.0;
	final static double SPEED_AVERAGE = 60.0;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO
		nodes = new HashMap<GeographicPoint, Node>();
		edges = new HashSet<Edge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		// TODO
		return nodes.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// TODO
		return nodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		// TODO
		return edges.size();
	}

		
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO
		if (location == null || nodes.containsKey(location)) {
			return false;
		}
		nodes.put(location, new Node(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		// TODO
		if (from == null || to == null || roadName == null || roadType == null 
				|| !nodes.containsKey(from) || !nodes.containsKey(to) ||
				length < 0) {
			throw new IllegalArgumentException("Illegal arguments for addEdge");
		}
	
		Node fromNode = nodes.get(from);
		Node toNode = nodes.get(to); 
		Edge newEdge = new Edge(fromNode, toNode, roadName, roadType, length);
		edges.add(newEdge);
		fromNode.addEdge(newEdge);
	}
	
	// Get a set of neighbor nodes from a Node.
	private Set<Node> getNeighbors(Node node) {
		return node.getNeighbors();
	}
	
	// Get the edge from startNode to endNode.
	private double getDistance(Node startNode, Node endNode) {
		// TODO
		for (Edge edge : edges) {
			if (edge.getStartNode().equals(startNode) &&
					edge.getEndNode().equals(endNode)) {
				return edge.getLength();
			}
		}
		return Double.POSITIVE_INFINITY;
	}
	
	public double getTime(Node startNode, Node endNode) {
		// TODO
		double distance = getDistance(startNode, endNode);
		for (Edge edge : edges) {
			if (edge.getStartNode().equals(startNode) &&
					edge.getEndNode().equals(endNode)) {
				if (edge.getRoadType().contains("motorway")) {
					return distance / MapGraph.SPEED_MOTORWAY;
				} else {
					return distance / MapGraph.SPEED_AVERAGE;
				}
			}
		}
		return 0.0;
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO
		Node startNode = nodes.get(start);
		Node goalNode = nodes.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists!");
			return null;
		}
		
		HashMap<Node, Node> parentMap = new HashMap<Node, Node>();
		boolean found = bfsSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path exists!");
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	// Return the list of intersections that form the shortest path from 
	// start to goal, given the starting node, the goal node and the map
	// containing the parent of each visited node.
	private List<GeographicPoint> constructPath(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap) {
		
		// TODO
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		Node currentNode = goalNode;
		while (currentNode != startNode) {
			path.addFirst(currentNode.getLocation());
			currentNode = parentMap.get(currentNode);
		}
		path.addFirst(startNode.getLocation());
		return path;
	}
	
	// Do Breadth First Search, given the starting node and the goal node and 
	// save the parent node for each visited node.
	// Return true if a path exists, false otherwise.
	private boolean bfsSearch(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		// TODO
		Queue<Node> toExplore = new LinkedList<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		toExplore.add(startNode);
		visited.add(startNode);
		boolean found = false;
			
		// Do the search	
		while (!toExplore.isEmpty()) {
			Node currentNode = toExplore.remove();
			if (currentNode == goalNode) {
				found = true;
				break;
			}
			Set<Node> neighbors = getNeighbors(currentNode);			
			for (Node neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, currentNode);
					toExplore.add(neighbor);
					nodeSearched.accept(neighbor.getLocation());
				}
			}
		}
		return found;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO
		Node startNode = nodes.get(start);
		Node goalNode = nodes.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists!");
			return null;
		}
		
		HashMap<Node, Node> parentMap = new HashMap<Node, Node>();
		boolean found = dijkstraSearch(startNode, goalNode, parentMap, 
				nodeSearched);
		
		if (!found) {
			System.out.println("No path exists!");
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	
	// Do Dijkstra Search, given the starting node and the goal node and 
	// save the parent node for each visited node.
	// Return true if a path exists, false otherwise.
	private boolean dijkstraSearch(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		// TODO
		PriorityQueue<Node> toExplore = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		for (GeographicPoint point : getVertices()) {
			Node n = nodes.get(point);
			n.setScore(Double.POSITIVE_INFINITY);
		}
		
		startNode.setScore(0.0);
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
			
		// Do the search	
		while (!toExplore.isEmpty()) {
			Node currentNode = toExplore.remove();
			count++;
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode.equals(goalNode)) {
					found = true;
					System.out.println("Dijkstra: " + count);
					break;
				}
			}
			Set<Node> neighbors = getNeighbors(currentNode);			
			for (Node neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					double newDistance = currentNode.getScore();
					
					getDistance(currentNode, neighbor);					
					if (newDistance < neighbor.getScore()) {
						neighbor.setScore(newDistance);
						parentMap.put(neighbor, currentNode);
						toExplore.add(neighbor);
						nodeSearched.accept(neighbor.getLocation());
					}
				}
			}
		}
		return found;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO
		Node startNode = nodes.get(start);
		Node goalNode = nodes.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists!");
			return null;
		}
		
		HashMap<Node, Node> parentMap = new HashMap<Node, Node>();
		boolean found = aStarSearch(startNode, goalNode, parentMap, 
				nodeSearched);
		
		if (!found) {
			System.out.println("No path exists!");
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	// Do A* Search, given the starting node and the goal node and 
	// save the parent node for each visited node.
	// Return true if a path exists, false otherwise.
	private boolean aStarSearch(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		// TODO
		PriorityQueue<Node> toExplore = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		for (GeographicPoint point : getVertices()) {
			Node n = nodes.get(point);
			n.setActualScore(Double.POSITIVE_INFINITY);
			n.setScore(Double.POSITIVE_INFINITY);
		}
		
		startNode.setActualScore(0.0);
		double distanceToGoal = startNode.getDistanceTo(goalNode);
		startNode.setScore(distanceToGoal);
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
			
		// Do the search	
		while (!toExplore.isEmpty()) {
			Node currentNode = toExplore.remove();
			count++;
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode.equals(goalNode)) {
					found = true;
					System.out.println("A*: " + count);
					break;
				}
			}
			Set<Node> neighbors = getNeighbors(currentNode);			
			for (Node neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					double newActualDistance = currentNode.getActualScore() +
							getDistance(currentNode, neighbor);					
					if (newActualDistance < neighbor.getActualScore()) {
						neighbor.setActualScore(newActualDistance);
						double newDistance = newActualDistance + 
								neighbor.getDistanceTo(goalNode);
						neighbor.setScore(newDistance);
						parentMap.put(neighbor, currentNode);
						toExplore.add(neighbor);
						nodeSearched.accept(neighbor.getLocation());
					}
				}
			}
		}
		return found;
	}
	
	/** Find the path from start to goal using the modified Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstraWithTime(GeographicPoint start, 
			GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstraWithTime(start, goal, temp);
	}

	/** Find the path from start to goal using the modified Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstraWithTime(GeographicPoint start, 
				GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO
		Node startNode = nodes.get(start);
		Node goalNode = nodes.get(goal);
		
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists!");
			return null;
		}
		
		HashMap<Node, Node> parentMap = new HashMap<Node, Node>();
		boolean found = dijkstraSearchWithTime(startNode, goalNode, parentMap, 
				nodeSearched);
		
		if (!found) {
			System.out.println("No path exists!");
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	// Do Dijkstra Search, given the starting node and the goal node and 
	// save the parent node for each visited node.
	// Return true if a path exists, false otherwise.
	private boolean dijkstraSearchWithTime(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap, Consumer<GeographicPoint> nodeSearched) {
			
		// TODO
		PriorityQueue<Node> toExplore = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		for (GeographicPoint point : getVertices()) {
			Node n = nodes.get(point);
			n.setScore(Double.POSITIVE_INFINITY);
		}
			
		startNode.setScore(0.0);
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
				
		// Do the search	
		while (!toExplore.isEmpty()) {
			Node currentNode = toExplore.remove();
			count++;
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode.equals(goalNode)) {
					found = true;
					System.out.println("Dijkstra: " + count);
					break;
				}
			}
			Set<Node> neighbors = getNeighbors(currentNode);			
			for (Node neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					double newScore = currentNode.getScore() + 
							getTime(currentNode, neighbor);
																	
					if (newScore < neighbor.getScore()) {
						neighbor.setScore(newScore);
						parentMap.put(neighbor, currentNode);
						toExplore.add(neighbor);
						nodeSearched.accept(neighbor.getLocation());
					}
				}
			}
		}
		return found;
	}
		
	/** Find the path from start to goal using the modified A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearchWithTime(GeographicPoint start, 
			GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}
		
	/** Find the path from start to goal using the modified A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearchWithTime(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO
		Node startNode = nodes.get(start);
		Node goalNode = nodes.get(goal);
			
		if (startNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists!");
			return null;
		}
			
		HashMap<Node, Node> parentMap = new HashMap<Node, Node>();
		boolean found = aStarSearch(startNode, goalNode, parentMap, 
				nodeSearched);
			
		if (!found) {
			System.out.println("No path exists!");
			return null;
		}
			
		return constructPath(startNode, goalNode, parentMap);
	}
		
	// Do the modified A* Search, given the starting node and the goal node and 
	// save the parent node for each visited node.
	// Return true if a path exists, false otherwise.
	private boolean aStarSearchWithTime(Node startNode, Node goalNode, 
			HashMap<Node, Node> parentMap, Consumer<GeographicPoint> nodeSearched) {
			
		PriorityQueue<Node> toExplore = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		for (GeographicPoint point : getVertices()) {
			Node n = nodes.get(point);
			n.setActualScore(Double.POSITIVE_INFINITY);
			n.setScore(Double.POSITIVE_INFINITY);
		}
			
		startNode.setActualScore(0.0);
		double scoreToGoal = startNode.getTimeTo(goalNode);
		startNode.setScore(scoreToGoal);
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
		
		// Do the search	
		while (!toExplore.isEmpty()) {
			Node currentNode = toExplore.remove();
			count++;
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				if (currentNode.equals(goalNode)) {
					found = true;
					System.out.println("A*: " + count);
					break;
				}
			}
			Set<Node> neighbors = getNeighbors(currentNode);			
			for (Node neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					double newActualScore = currentNode.getActualScore() +
							getTime(currentNode, neighbor);					
					if (newActualScore < neighbor.getActualScore()) {
						double newScore = newActualScore + 
								neighbor.getTimeTo(goalNode);
						neighbor.setActualScore(newActualScore);
						neighbor.setScore(newScore);
						parentMap.put(neighbor, currentNode);
						toExplore.add(neighbor);
						nodeSearched.accept(neighbor.getLocation());
					}
				}
			}
		}
		return found;
	}
	
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		*/
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
}
