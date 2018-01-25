package aStar;

import java.util.ArrayList;
import java.util.Collections;

/**
 * The AreaMap class defines a graph representing a map of Duluth, MN.
 * Implements Comparable interface to allow sorting, requires compareTo() method
 *
 * This does not need to be modified by students
 *
 * @author Original author: Tom Gibbons.
 * @version Spring 2018 version
 */
public class Node implements Comparable<Node> {

    AreaMap map;                                        // Map that this is a part of
    ArrayList<Node> neighborList;                       // list of neighbor nodes to this node
    ArrayList<Double> neighborDistance;			// distance to each of the neighbor nodes in above list
    boolean visited;
    public double distanceFromStart;			// distance from Start node to this node
    public double heuristicDistanceToGoal;		// estimated distance from node to Goal node
    public double TotalDistance;                        // estimated distance from start to Goal node going through this node, TotalDistanceFromGoal = heuristicDistanceToGoal + distanceFromStart
    Node previousNode;					// previous node in the path to this node.  How did we get to this node.
    public int nodeNum;					// the number of this node on the map
    public double x;					// x coordinate of this node
    public double y;					// y coordinate of this node
    boolean isStart;					// is this a start node?
    boolean isGoal;						// is this a goal node?

    /**
     * This constructor creates the node that is not a start or goal and sets distance from start to max
     */
    Node(int n, double x, double y) {
        neighborList = new ArrayList<Node>();
        neighborDistance = new ArrayList<Double>();
        this.nodeNum = n;
        this.x = x;
        this.y = y;
        this.visited = false;
        this.distanceFromStart = Integer.MAX_VALUE;
        this.isStart = false;
        this.isGoal = false;
    }
    /**
     * This constructor creates the node with all the given parameters
     */
    Node(double x, double y, boolean visited, int distanceFromStart, boolean isStart, boolean isGoal) {
        neighborList = new ArrayList<Node>();
        this.x = x;
        this.y = y;
        this.visited = visited;
        this.distanceFromStart = distanceFromStart;
        this.isStart = isStart;
        this.isGoal = isGoal;
    }
    /**
     * getNeighborDistance() returns the distance the the i'th neighbor
     * @param i is the index for the neighbor. This is not checked for bounds
     */
    public Double getNeighborDistance(int i) {
        return neighborDistance.get(i);
    }
    /**
     * getNeighborSize() returns number of neighbors
     */
    public int getNeighborSize() {
        return neighborList.size();
    }

    public Node getNeighbor(int i) {
        return neighborList.get(i);
    }

    public void setNeighbor(Node n, double dist) {
        neighborList.add(n);
        neighborDistance.add(dist);
    }

    public boolean isVisited() {
        return visited;
    }

    public void setVisited(boolean visited) {
        this.visited = visited;
    }

    public Node getPreviousNode() {
        return previousNode;
    }

    public void setPreviousNode(Node previousNode) {
        this.previousNode = previousNode;
    }

    public boolean isStart() {
        return isStart;
    }

    public void setStart(boolean isStart) {
        this.isStart = isStart;
    }

    public boolean isGoal() {
        return isGoal;
    }

    public void setGoal(boolean isGoal) {
        this.isGoal = isGoal;
    }

    public boolean equals(Node node) {
        return (node.x == x) && (node.y == y);
    }
    /**
     * compareTo() Compare this node with another node using TotalDistanceFromGoal.
     *        Used by Collections.sort to sort the list in order by total distance
     * @param otherNode is the node to compare this node to
     * @return a int where -1 is when this node has less total distance, 
     *                     +1 is when this node has larger total distance
     */
    public int compareTo(Node otherNode) {
        if (TotalDistance < otherNode.TotalDistance) {
            return -1;
        } else if (TotalDistance > otherNode.TotalDistance) {
            return 1;
        } else {
            return 0;
        }
    }
}
