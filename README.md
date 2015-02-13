# eecs391_pr01
A-star agent implementation for EECS 391 at CWRU, Spring 2015.

Team: Shaun Howard (smh150) and Matt Swartwout (mws85)

Our A* agent utilizes a priority queue to compare map locations based on cost. The hope is that this implementation
will lead to a favorable runtime that eliminates the linear time comparison of sets or lists.

An agent has their own inner agent map class where they keep track of various locations necessary to planning. The code is well-documented,
but the agent map contains the start and goal locations as well as the enemy and resource locations.
The map also holds the x and y extents of itself.

The map location inner class resembles a node in the A* search. This node has x and y coordinates on the map, a distance
from the beginning of the current search, a cost which is the distance from beginning of this search added to the heuristic,
and a previously visited map location. The map location class implements the comparable interface so that the priority
queue can compare map locations based on their cost and keep the minimum cost node at the top of the queue.

Any values that were calculated but not reused were eliminated and not stored in order to remain efficient. Null-checking
was still utilized in case values were null on different maps even though they would most likely not be.

The heuristic used was the Chebyshev distance as implemented in the SEPIA util package class DistanceMetrics.

All together, the implementation runs pretty fast while maintaining accuracy for different maps and the code is
well-documented. Please ask either of us if you have any questions.