package roadgraph;

import java.util.*;

import geography.GeographicPoint;

public class MapNode {
	
	private GeographicPoint point;
	private List<MapEdge> edges;
	private List<MapNode> neighbors;
	
	public MapNode(GeographicPoint point) {
		this.point = point;
		edges = new ArrayList<MapEdge>();
		neighbors = new ArrayList<MapNode>();
	}

	public void addEdge(MapEdge edge) {
		this.edges.add(edge);
	}
	
	public void addNeighbor(MapNode node) {
		this.neighbors.add(node);
	}
	
	public List<MapNode> getNeighbors() {
		return this.neighbors;
	}
	
	public GeographicPoint getLocation() {
		return this.point;
	}
	
	
}