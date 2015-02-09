package edu.cwru.sepia.agent;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.ResourceNode;
import edu.cwru.sepia.environment.model.state.State;
import edu.cwru.sepia.environment.model.state.Unit;
import edu.cwru.sepia.util.Direction;
import edu.cwru.sepia.util.DistanceMetrics;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

public class AstarAgent extends Agent {

    AgentMap agentMap;

    class MapLocation implements Comparable<MapLocation>{

        // The x and y coordinates of this location.
        int x = 0, y = 0;

        // The previously visited map location.
        MapLocation previous = null;

        // The cost to reach this location.
        float cost = 0;

        // The distance of this location from the initial location.
        float distanceFromBeginning = 0;

        // The estimated distance to the end of the path.
        float heuristic = 0;

        // A map of neighbors, reachable and unreachable.
        Map<MapLocation, Boolean> neighbors = new HashMap<>();

        /**
         * Gets the x-coordinate of this location.
         *
         * @return the x-coordinate of this location
         */
        public int getX() {
            return x;
        }

        /**
         * Gets the y-coordinate of this location.
         *
         * @return the y-coordinate of this location
         */
        public int getY() {
            return y;
        }

        /**
         * Constructor for a map location based on x,y coordinates,
         * a previously visited map location, and the cost to reach
         * this location.
         *
         * @param x - the x coordinate on the map of this location
         * @param y - the y coordinate on the map of this location
         * @param previous - the previously visited location
         * @param cost - - the cost to reach this location
         */
        public MapLocation(int x, int y, MapLocation previous, float cost)
        {
            this.x = x;
            this.y = y;
            this.previous = previous;
            this.cost = cost;
        }

        /**
         * Sets the previously visited location.
         *
         * @param previous - the previously visited location
         */
        public void setPrevious(MapLocation previous) {
            this.previous = previous;
        }

        /**
         * Gets the previously visited location.
         *
         * @return the previously visited location
         */
        public MapLocation getPrevious() {
            return previous;
        }

        /**
         * Sets the distance from the beginning location.
         *
         * @param distanceFromBeginning - the distance from the beginning location
         */
        public void setDistanceFromBeginning(float distanceFromBeginning) {
            this.distanceFromBeginning = distanceFromBeginning;
        }

        /**
         * Gets the distance from the beginning location.
         *
         * @return the distance from the beginning location
         */
        public float getDistanceFromBeginning() {
            return distanceFromBeginning;
        }


        /**
         * Sets the heuristic distance to the end location.
         *
         * @param heuristic - the heuristic distance to the end location
         */
        public void setHeuristic(float heuristic) {
            this.heuristic = heuristic;
        }

        /**
         * Gets the heuristic distance to the end location.
         *
         * @return the heuristic distance to the end location
         */
        public float getHeuristic() {
            return heuristic;
        }

        /**
         * Adds a neighbor location to this location.
         *
         * @param neighbor - the neighbor location to add
         * @param isConnected - whether the neighbor is connected
         */
        public void addNeighbor(MapLocation neighbor, boolean isConnected) {
            if (neighbor != null) {
                this.neighbors.put(neighbor, isConnected);
            }
        }

        /**
         * Gets the neighbor locations of this location.
         *
         * @return the neighbor locations of this location
         */
        public Map<MapLocation, Boolean> getNeighbors() {
            return neighbors;
        }

        /**
         * Gets a set of neighbors that are currently reachable from this
         * current location.
         *
         * @param enemyLocation - the location of the enemy footman who blocks the path
         * @param resourceLocations - the set of resource locations which are blocking the path
         * @return connected neighbors of this location
         */
        public Set<MapLocation> getReachableNeighbors(MapLocation enemyLocation, Set<MapLocation> resourceLocations, AgentMap map) {
        	System.out.println("Getting reachable neighbors");
        	if (enemyLocation == null)
        		System.out.println("enemyLoc is null");
        	if (resourceLocations == null)
        		System.out.println("resourceLoc is null");
        	if (map == null)
        		System.out.println("map is null");
        	
            Set<MapLocation> locations = getNeighbors(map);
            	System.out.println("# of neighbors found is " +locations.size());
            Iterator<MapLocation> locationItr = locations.iterator();
            MapLocation curr = null;
            // Remove any neighbors not reachable from this location.
            while (locationItr.hasNext()) {
                curr = locationItr.next();
                if (resourceLocations.contains(curr) || (enemyLocation != null && enemyLocation.equals(curr))) {
                    locationItr.remove();
                }
                else
                	System.out.println("Reachable neighbor found at " +curr.toString());
            }
            
            System.out.println("# of reachable neighbors is " +locations.size());
            return locations;
        }

        /**
         * Gets the neighbors map locations of this map location as long as they
         * exist on the given agent map.
         *
         * @param map - the agent map to search for neighbor locations of this location in
         * @return the set of map locations neighboring this map location
         */
        public Set<MapLocation> getNeighbors(AgentMap map){
            Set<MapLocation> neighbors = new HashSet<>();
            neighbors.add(getNorthNeighbor(map));
            neighbors.add(getSouthNeighbor(map));
            neighbors.add(getEastNeighbor(map));
            neighbors.add(getWestNeighbor(map));
            neighbors.add(getNorthEastNeighbor(map));
            neighbors.add(getNorthWestNeighbor(map));
            neighbors.add(getSouthEastNeighbor(map));
            neighbors.add(getSouthWestNeighbor(map));
            if (neighbors.contains(null)){
                neighbors.remove(null);
            }
            return neighbors;
        }

        /**
         * Returns the map location southwest of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the southwest neighbor in
         * @return the southwest neighbor of this location
         */
        private MapLocation getSouthWestNeighbor(AgentMap map) {
            //x - 1 & y + 1
            int neighborX = this.x - 1;
            int neighborY = this.y + 1;

            // Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
                // Check if the neighbor is within the y extent of the map.
                if (map.getYExtent() > neighborY && neighborY >= 0){
                    return new MapLocation(neighborX, neighborY, this, 0);
                }
            }
            return null;
        }

        /**
         * Returns the map location southeast of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the southeast neighbor in
         * @return the southeast neighbor of this location
         */
        private MapLocation getSouthEastNeighbor(AgentMap map) {
            //x + 1 & y + 1
            int neighborX = this.x + 1;
            int neighborY = this.y + 1;
            
            //Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
            	//Check if the neighbor is within the y extent of the map
            	if (map.getYExtent() > neighborY && neighborY > 0) {
            		return new MapLocation(neighborX, neighborY, this, 0);
            	}
            }
            
            return null;
        }

        /**
         * Returns the map location northwest of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the northwest neighbor in
         * @return the northwest neighbor of this location
         */
        private MapLocation getNorthWestNeighbor(AgentMap map) {
            //x - 1 & y - 1
        	int neighborX = this.x - 1;
            int neighborY = this.y - 1;
            
            //Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
            	//Check if the neighbor is within the y extent of the map
            	if (map.getYExtent() > neighborY && neighborY > 0) {
            		return new MapLocation(neighborX, neighborY, this, 0);
            	}
            }
            
            return null;
        }

        /**
         * Returns the map location northeast of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the northeast neighbor in
         * @return the northeast neighbor of this location
         */
        private MapLocation getNorthEastNeighbor(AgentMap map) {
            //x + 1 & y - 1
        	int neighborX = this.x + 1;
            int neighborY = this.y - 1;
            
            //Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
            	//Check if the neighbor is within the y extent of the map
            	if (map.getYExtent() > neighborY && neighborY > 0) {
            		return new MapLocation(neighborX, neighborY, this, 0);
            	}
            }
            
            return null;
        }

        /**
         * Returns the map location west of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the west neighbor in
         * @return the west neighbor of this location
         */
        private MapLocation getWestNeighbor(AgentMap map) {
            //x - 1 & y
        	int neighborX = this.x - 1;
            
            //Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
            		return new MapLocation(neighborX, this.y, this, 0);
            	}
          
            return null;
        }

        /**
         * Returns the map location east of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the east neighbor in
         * @return the east neighbor of this location
         */
        private MapLocation getEastNeighbor(AgentMap map) {
            //x + 1 & y
        	int neighborX = this.x + 1;
            
            //Check if the neighbor is within the x extent of the map.
            if (map.getXExtent() > neighborX && neighborX >= 0) {
            		return new MapLocation(neighborX, this.y, this, 0);
            	}
          
            return null;
        }

        /**
         * Returns the map location south of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the south neighbor in
         * @return the south neighbor of this location
         */
        private MapLocation getSouthNeighbor(AgentMap map) {
            //x & y + 1
        	int neighborY = this.y + 1;
            
            //Check if the neighbor is within the y extent of the map.
            if (map.getYExtent() > neighborY && neighborY >= 0) {
            		return new MapLocation(this.x, neighborY, this, 0);
            	}
          
            return null;
        }

        /**
         * Returns the map location north of this map location
         * or null if the location is off of the map.
         *
         * @param map - the map to find the north neighbor in
         * @return the north neighbor of this location
         */
        private MapLocation getNorthNeighbor(AgentMap map) {
            //x & y - 1
        	int neighborY = this.y - 1;
            
            //Check if the neighbor is within the y extent of the map.
            if (map.getYExtent() > neighborY && neighborY >= 0) {
            		return new MapLocation(this.x, neighborY, this, 0);
            	}
          
            return null;
        }

        /**
         * Sets the cost of this location.
         *
         * @param cost - the cost of this location
         */
        public void setCost(float cost) {
            this.cost = cost;
        }

        /**
         * Gets the cost of this location.
         *
         * @return the cost of this location
         */
        public float getCost() {
            return cost;
        }

        /**
         * Compares locations by their cost. Utilizes
         * the Java compareTo() for Floats.
         *
         * @param location - the location to compare to this location
         * @return the comparison of the locations by cost
         */
        @Override
        final public int compareTo(MapLocation location) {
            Float thisCost = new Float(this.getCost());
            Float locationCost = new Float(location.getCost());
            return thisCost.compareTo(locationCost);
        }

        /**
         * Gets the coordinates of this location as a string.
         *
         * @return - the coordinates <x,y> of this location as a string
         */
        public String getCoordinateString() {
            return "<" + this.x + ", " + this.y + ">";
        }

        /**
         * Determines whether two locations are the same based on their costs,
         * x, and y coordinates.
         *
         * @param location - the location to compare with this location
         * @return whether the the locations are the same
         */
        public boolean sameLocation(MapLocation location) {
            if (this.x == location.x
                    && this.y == location.y
                    //Removing this for now since there is no difference in path costs currently
                    //&& this.getCost() == location.getCost()
                    ) {
                return true;
            }
            return false;
        }

        /**
         * Determines whether two locations are equal based on their costs,
         * x, and y coordinates.
         *
         * @param o - the object to compare with this location
         * @return whether the input object is equivalent to this location
         */
        @Override
        public boolean equals(Object o) {
            if (o != null && o instanceof MapLocation) {
                MapLocation l = (MapLocation) o;
                if (sameLocation(l)) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Hash code for this location. Generated based on
         * x, y coordinates and cost of this location.
         *
         * @return the hash code of this location
         */
        @Override
        public int hashCode() {
            int result = (cost != +0.0f ? Float.floatToIntBits(cost) : 0);
            result = 31 * result + x;
            result = 31 * result + y;
            return result;
        }

    }

    public class AgentMap {
        /* the length of the maze by rows. */
        int xExtent = 0;
        /* the width of the maze by columns. */
        int yExtent = 0;
        /* the size of the maze by length times width. */
        int size = 0;
        /* the beginning location of the maze. */
        MapLocation begin = null;
        /* the ending location of the maze. */
        MapLocation end = null;

        /* The location of the enemy footman on this map. */
        MapLocation enemyFootmanLoc;

        Set<MapLocation> resourceLocations = new HashSet<>();

        public AgentMap(int xExtent, int yExtent, MapLocation agentStart, MapLocation agentStop, Set<MapLocation> resourceLocations){
            this.xExtent = xExtent;
            this.yExtent = yExtent;
            this.begin = agentStart;
            this.end = agentStop;
            this.resourceLocations = resourceLocations;
        }

        /**
         * Gets the beginning location of this maze.
         *
         * @return the beginning location of this maze
         */
        public MapLocation getBegin() {
            return begin;
        }

        /**
         * Gets the size of this maze.
         *
         * @return the size of this maze
         */
        public int size() {
            return this.size;
        }
        /**
         * Gets the x extent of the maze.
         *
         * @return the x extent of the maze
         */
        public int getXExtent() {
            return this.xExtent;
        }
        /**
         * Gets the y extent of the maze.
         *
         * @return the y extent of the maze
         */
        public int getYExtent() {
            return this.yExtent;
        }

        public void setEnemyLocation(MapLocation enemyFootmanLoc) {
            this.enemyFootmanLoc = enemyFootmanLoc;
        }

        public MapLocation getEnemyLocation(){
            return this.enemyFootmanLoc;
        }
    }

    Stack<MapLocation> path;
    int footmanID, townhallID, enemyFootmanID;
    MapLocation nextLoc;

    private long totalPlanTime = 0; // nsecs
    private long totalExecutionTime = 0; //nsecs

    public AstarAgent(int playernum)
    {
        super(playernum);

        System.out.println("Constructed AstarAgent");
    }

    @Override
    public Map<Integer, Action> initialStep(State.StateView newstate, History.HistoryView statehistory) {
        // get the footman location
        List<Integer> unitIDs = newstate.getUnitIds(playernum);

        if(unitIDs.size() == 0)
        {
            System.err.println("No units found!");
            return null;
        }

        footmanID = unitIDs.get(0);

        // double check that this is a footman
        if(!newstate.getUnit(footmanID).getTemplateView().getName().equals("Footman"))
        {
            System.err.println("Footman unit not found");
            return null;
        }

        // find the enemy playernum
        Integer[] playerNums = newstate.getPlayerNumbers();
        int enemyPlayerNum = -1;
        for(Integer playerNum : playerNums)
        {
            if(playerNum != playernum) {
                enemyPlayerNum = playerNum;
                break;
            }
        }

        if(enemyPlayerNum == -1)
        {
            System.err.println("Failed to get enemy playernumber");
            return null;
        }

        // find the townhall ID
        List<Integer> enemyUnitIDs = newstate.getUnitIds(enemyPlayerNum);

        if(enemyUnitIDs.size() == 0)
        {
            System.err.println("Failed to find enemy units");
            return null;
        }

        townhallID = -1;
        enemyFootmanID = -1;
        for(Integer unitID : enemyUnitIDs)
        {
            Unit.UnitView tempUnit = newstate.getUnit(unitID);
            String unitType = tempUnit.getTemplateView().getName().toLowerCase();
            if(unitType.equals("townhall"))
            {
                townhallID = unitID;
            }
            else if(unitType.equals("footman"))
            {
                enemyFootmanID = unitID;
            }
            else
            {
                System.err.println("Unknown unit type");
            }
        }

        if(townhallID == -1) {
            System.err.println("Error: Couldn't find townhall");
            return null;
        }

        long startTime = System.nanoTime();
        path = findPath(newstate);
        totalPlanTime += System.nanoTime() - startTime;

        return middleStep(newstate, statehistory);
    }

    @Override
    public Map<Integer, Action> middleStep(State.StateView newstate, History.HistoryView statehistory) {
    	
        long startTime = System.nanoTime();
        long planTime = 0;

        Map<Integer, Action> actions = new HashMap<Integer, Action>();

        if(shouldReplanPath(newstate, statehistory, path)) {
            long planStartTime = System.nanoTime();
            path = findPath(newstate);
            planTime = System.nanoTime() - planStartTime;
            totalPlanTime += planTime;
        }

        Unit.UnitView footmanUnit = newstate.getUnit(footmanID);

        int footmanX = footmanUnit.getXPosition();
        	System.out.println("Footman x = " +footmanX);
        int footmanY = footmanUnit.getYPosition();
        	System.out.println("Footman y = " +footmanY);
        
        if(path == null)
        	System.out.println("Path is null");
        if (nextLoc == null)
        	System.out.println("nextLoc is null");
        if(!path.empty() && (nextLoc == null || (footmanX == nextLoc.x && footmanY == nextLoc.y))) {

            // stat moving to the next step in the path
            nextLoc = path.pop();

            System.out.println("Moving to (" + nextLoc.x + ", " + nextLoc.y + ")");
        }

        if(nextLoc != null && (footmanX != nextLoc.x || footmanY != nextLoc.y))
        {
            int xDiff = nextLoc.x - footmanX;
            int yDiff = nextLoc.y - footmanY;

            // figure out the direction the footman needs to move in
            Direction nextDirection = getNextDirection(xDiff, yDiff);

            actions.put(footmanID, Action.createPrimitiveMove(footmanID, nextDirection));
        } else {
            Unit.UnitView townhallUnit = newstate.getUnit(townhallID);

            // if townhall was destroyed on the last turn
            if(townhallUnit == null) {
                terminalStep(newstate, statehistory);
                return actions;
            }

            if(Math.abs(footmanX - townhallUnit.getXPosition()) > 1 ||
                    Math.abs(footmanY - townhallUnit.getYPosition()) > 1)
            {
                System.err.println("Invalid plan. Cannot attack townhall");
                totalExecutionTime += System.nanoTime() - startTime - planTime;
                return actions;
            }
            else {
                System.out.println("Attacking TownHall");
                // if no more movements in the planned path then attack
                actions.put(footmanID, Action.createPrimitiveAttack(footmanID, townhallID));
            }
        }

        totalExecutionTime += System.nanoTime() - startTime - planTime;
        return actions;
    }

    @Override
    public void terminalStep(State.StateView newstate, History.HistoryView statehistory) {
        System.out.println("Total turns: " + newstate.getTurnNumber());
        System.out.println("Total planning time: " + totalPlanTime/1e9);
        System.out.println("Total execution time: " + totalExecutionTime/1e9);
        System.out.println("Total time: " + (totalExecutionTime + totalPlanTime)/1e9);
    }

    @Override
    public void savePlayerData(OutputStream os) {

    }

    @Override
    public void loadPlayerData(InputStream is) {

    }

    /**
     * You will implement this method.
     *
     * This method should return true when the path needs to be replanned
     * and false otherwise. This will be necessary on the dynamic map where the
     * footman will move to block your unit.
     *
     * @param state
     * @param history
     * @param currentPath
     * @return
     */
    private boolean shouldReplanPath(State.StateView state, History.HistoryView history, Stack<MapLocation> currentPath)
    {
       // Unit.UnitView footmanUnit = state.getUnit(footmanID);
       // MapLocation footmanLocation = new MapLocation(footmanUnit.getXPosition(), footmanUnit.getYPosition(), null, 0);
        MapLocation enemyLocation;

        if(enemyFootmanID != -1) {
            Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
            enemyLocation = new MapLocation(enemyFootmanUnit.getXPosition(), enemyFootmanUnit.getYPosition(), null, 0);
            //check if enemy is on path (via map location) and close to player's next move (i.e. distance <= 2)
            return enemyBlockingPath(enemyLocation, currentPath);
        }
        return false;
    }

    //Wish there was a way to check if enemy was on path
    private boolean enemyBlockingPath(MapLocation enemyLocation, Stack<MapLocation> currentPath){
        MapLocation nextLocation = currentPath.peek();
        MapLocation subsequentLocation = currentPath.elementAt(currentPath.size() - 2);
        float enemyToNextLocation = distanceBetweenLocations(nextLocation, enemyLocation);
        float enemyToSubsequentLocation = distanceBetweenLocations(subsequentLocation, enemyLocation);

        //check if enemy is near the next location or subsequent location of footman along path
        if (enemyToNextLocation <= 1 || enemyToSubsequentLocation <= 1){
            return true;
        }
        return false;
    }

    /**
     * This method is implemented for you. You should look at it to see examples of
     * how to find units and resources in Sepia.
     *
     * @param state
     * @return
     */
    private Stack<MapLocation> findPath(State.StateView state)
    {
        Unit.UnitView townhallUnit = state.getUnit(townhallID);
        Unit.UnitView footmanUnit = state.getUnit(footmanID);

        MapLocation startLoc = new MapLocation(footmanUnit.getXPosition(), footmanUnit.getYPosition(), null, 0);

        MapLocation goalLoc = new MapLocation(townhallUnit.getXPosition(), townhallUnit.getYPosition(), null, 0);

        MapLocation footmanLoc = null;
        if(enemyFootmanID != -1) {
            Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
            footmanLoc = new MapLocation(enemyFootmanUnit.getXPosition(), enemyFootmanUnit.getYPosition(), null, 0);
        }

        // get resource locations
        List<Integer> resourceIDs = state.getAllResourceIds();
        Set<MapLocation> resourceLocations = new HashSet<MapLocation>();
        for(Integer resourceID : resourceIDs)
        {
            ResourceNode.ResourceView resource = state.getResourceNode(resourceID);

            resourceLocations.add(new MapLocation(resource.getXPosition(), resource.getYPosition(), null, 0));
        }

        return AstarSearch(startLoc, goalLoc, state.getXExtent(), state.getYExtent(), footmanLoc, resourceLocations);
    }

    /**
     * This is the method you will implement for the assignment. Your implementation
     * will use the A* algorithm to compute the optimum path from the start position to
     * a position adjacent to the goal position.
     * <p/>
     * You will return a Stack of positions with the top of the stack being the first space to move to
     * and the bottom of the stack being the last space to move to. If there is no path to the townhall
     * then return null from the method and the agent will print a message and do nothing.
     * The code to execute the plan is provided for you in the middleStep method.
     * <p/>
     * As an example consider the following simple map
     * <p/>
     * F - - - -
     * x x x - x
     * H - - - -
     * <p/>
     * F is the footman
     * H is the townhall
     * x's are occupied spaces
     * <p/>
     * xExtent would be 5 for this map with valid X coordinates in the range of [0, 4]
     * x=0 is the left most column and x=4 is the right most column
     * <p/>
     * yExtent would be 3 for this map with valid Y coordinates in the range of [0, 2]
     * y=0 is the top most row and y=2 is the bottom most row
     * <p/>
     * resourceLocations would be {(0,1), (1,1), (2,1), (4,1)}
     * <p/>
     * The path would be
     * <p/>
     * (1,0)
     * (2,0)
     * (3,1)
     * (2,2)
     * (1,2)
     * <p/>
     * Notice how the initial footman position and the townhall position are not included in the path stack
     *
     * @param start             Starting position of the footman
     * @param goal              MapLocation of the townhall
     * @param xExtent           Width of the map
     * @param yExtent           Length of the map
     * @param resourceLocations Set of positions occupied by resources
     * @return Stack of positions with top of stack being first move in plan
     */
    private Stack<MapLocation> AstarSearch(MapLocation start, MapLocation goal, int xExtent, int yExtent, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations) {
        System.out.println("AStarSearch Started");
  
    	Set<MapLocation> expandedLocations = new HashSet<>();
        PriorityQueue<MapLocation> openLocations = new PriorityQueue<>();
        agentMap = new AgentMap(xExtent, yExtent, start, goal, resourceLocations);
        agentMap.setEnemyLocation(enemyFootmanLoc);

        openLocations.add(start);
        while (!openLocations.isEmpty()) {
            MapLocation cheapestLocation = openLocations.poll();
            System.out.println("Cheapest location is " +cheapestLocation.getCoordinateString());
            if (cheapestLocation.equals(goal)) {
            	System.out.println("AStarSearch Complete. Found the goal state at " +cheapestLocation.getCoordinateString());
                return AstarPath(cheapestLocation);
            }

            //Need to properly implement getting neighbors with fast runtime
            Set<MapLocation> possibleLocations = cheapestLocation.getReachableNeighbors(enemyFootmanLoc, resourceLocations, agentMap);

            for (MapLocation location : possibleLocations) {
                if (!expandedLocations.contains(location)) { //this may have to be amended
                    location.setPrevious(cheapestLocation);
                    location.setDistanceFromBeginning(distanceBetweenLocations(start, location));
                    location.setHeuristic(distanceBetweenLocations(location, goal));
                    location.setCost(location.getDistanceFromBeginning() + location.getHeuristic());
                    expandedLocations.add(cheapestLocation);
                    openLocations.add(location);
                }
            }
        }
        System.out.println("AStarSearch completed, no path found");
        // return an empty path
        return null;
    }

    //Chebyshev distance
    private float distanceBetweenLocations(MapLocation beginning, MapLocation end) {
        if (beginning != null && end != null) {
            return DistanceMetrics.chebyshevDistance(beginning.x, beginning.y, end.x, end.y);
        }
        return 0;
    }

    /**
     * Returns the A* path to the given end location
     * from the beginning location of the map.
     *
     * @param end - the location to get the A* path to
     * from the beginning location
     * @return the stack of locations from the beginning of the
     * map (top of stack) to the end of the map (bottom of stack)
     */
    public static Stack<MapLocation> AstarPath(MapLocation end) {

        Stack<MapLocation> astarPath = new Stack<>();

        MapLocation curr = end;

        astarPath.push(curr);

        while (curr.getPrevious() != null) {

            curr = curr.getPrevious();

            astarPath.push(curr);
        }

        return astarPath;
    }

    /**
     * Primitive actions take a direction (e.g. NORTH, NORTHEAST, etc)
     * This converts the difference between the current position and the
     * desired position to a direction.
     *
     * @param xDiff Integer equal to 1, 0 or -1
     * @param yDiff Integer equal to 1, 0 or -1
     * @return A Direction instance (e.g. SOUTHWEST) or null in the case of error
     */
    private Direction getNextDirection(int xDiff, int yDiff) {

        // figure out the direction the footman needs to move in
        if(xDiff == 1 && yDiff == 1)
        {
            return Direction.SOUTHEAST;
        }
        else if(xDiff == 1 && yDiff == 0)
        {
            return Direction.EAST;
        }
        else if(xDiff == 1 && yDiff == -1)
        {
            return Direction.NORTHEAST;
        }
        else if(xDiff == 0 && yDiff == 1)
        {
            return Direction.SOUTH;
        }
        else if(xDiff == 0 && yDiff == -1)
        {
            return Direction.NORTH;
        }
        else if(xDiff == -1 && yDiff == 1)
        {
            return Direction.SOUTHWEST;
        }
        else if(xDiff == -1 && yDiff == 0)
        {
            return Direction.WEST;
        }
        else if(xDiff == -1 && yDiff == -1)
        {
            return Direction.NORTHWEST;
        }

        System.err.println("Invalid path. Could not determine direction");
        return null;
    }
}
