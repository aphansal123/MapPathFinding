package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private double distance;
	private String roadType;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, String streetName, 
				   double distance, String roadType) {
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.distance = distance;
		this.roadType = roadType;
	}
	
	public GeographicPoint getEndPoint() {
		return this.end;
	}

}