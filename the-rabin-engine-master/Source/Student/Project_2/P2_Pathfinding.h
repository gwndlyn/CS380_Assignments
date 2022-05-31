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

    enum ONLIST
    {
        NONE,
        OPEN,
        CLOSED
    };

    struct Node
    {
        Vec3 pos;
        Vec3 parentNodePos;
        double fCost, gCost;
        ONLIST onList;

        Node(Vec3 p = Vec3(), Vec3 par = Vec3(), double f = FLT_MAX, double g = 0.0f, ONLIST ol = NONE);
    };
    
    //variables
    float heuristic;
    std::array<Node, 1600> nodeArr;
    std::vector<Node*> openList;
    std::vector<Node*> closedList;

    //reference varables 
    PathResult pathResult;
    PathRequest& req;
    Vec3 gridSize;

    //helper functions for nodes
    float CalculateHeuristicCost(Vec3 start, Vec3 end);
    void UpdateCost(Node* child, Node* parent, float newF, float newG);
    Node* PopCheapestOpenListNode();
    Node* FindInBothLists(const Vec3& pos);
    int SingleIndexConverter(const Vec3& pos);

    //algo functions
    void runASTAR();
    //void runFLOYD_WARSHALL();
    //void runGOAL_BOUNDING();
    //void runJPS_PLUS();

};