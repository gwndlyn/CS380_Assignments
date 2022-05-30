#pragma once
#include "Misc/PathfindingDetails.hpp"

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

    struct Node
    {
        Vec3 pos;
        double fCost, gCost, hCost;
        Vec3 parentPos;
        //float cost; //cost from start to that point
        //might have to write a operator< or operator> 
    };

    //using NodePQ = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
    using NodeVec = std::vector<Node>;
    
    float heuristic;
    NodeVec openList;
    NodeVec closedList;
    PathResult pathResult;
    PathRequest& req;

    float CalculateHeuristicCost(Vec3 start);

    void runASTAR();
    //void runFLOYD_WARSHALL();
    //void runGOAL_BOUNDING();
    //void runJPS_PLUS();

};