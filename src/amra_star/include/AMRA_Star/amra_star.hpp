#ifndef AMRA_STAR_HPP
#define AMRA_STAR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "AMRA_Star/ThreeDmap.h"
#include "AMRA_Star/HeuristicManager.h"
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <array>
#include <set>
#include <chrono>
#include <memory>


namespace AMRA_Star {
struct Node {
    std::array<int, 3> position; // x, y, z
    RotationQuaternion orientation;
    Resolution::Level res;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent; 

    Node(std::array<int, 3> pos, RotationQuaternion ori, Resolution::Level resolution, double gCost, double hCost, double fCost, std::shared_ptr<Node> parentNode)
        : position(pos), orientation(ori), res(resolution), g_cost(gCost), h_cost(hCost), f_cost(fCost), parent(parentNode) {}

    Node(const Node& other)
        : position(other.position), orientation(other.orientation), res(other.res), g_cost(other.g_cost), h_cost(other.h_cost), f_cost(other.f_cost), parent(other.parent) {}

    bool operator<(const Node& other) const {
        return f_cost < other.f_cost || (f_cost == other.f_cost && position < other.position);
    }
    bool operator==(const Node& other) const {
        return (position == other.position)&&(orientation == other.orientation);
    }
    void operator=(const std::shared_ptr<Node>& other){
        position=other->position; orientation == other->orientation; res = other->res; g_cost = other->g_cost; h_cost = other->h_cost; f_cost = other->f_cost;
    }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
        if (lhs->f_cost < rhs->f_cost) return true;
        if (rhs->f_cost < lhs->f_cost) return false;
        return lhs->position < rhs->position;
    }
};


class AMRAstar 
{
private:
    std::shared_ptr<Node> start, goal;
    ThreeDMap grid;
    std::vector<std::set<std::shared_ptr<Node>,NodeCompare>> openLists;
    std::vector<std::vector<std::shared_ptr<Node>>> closeList;
    std::vector<std::shared_ptr<Node>> INCONS;
    std::vector<Node> solutionPath;
    std::vector<std::shared_ptr<Node>> Explored;
    HeuristicManager manager;
    int weight1;
    int weight2;
    int weight1_init;
    int weight2_init;
    int weight1Step;
    int weight2step;
    std::vector<std::pair<Resolution::Level, int>> heurs_map;
    double totalDistance;
    double startTime;
    double searchTime;
    double timeLimit; //ms


    void addNodeToExplored(const std::shared_ptr<Node>& node) {
        auto it = std::find_if(Explored.begin(), Explored.end(), 
                           [&node](const std::shared_ptr<Node>& n) { return *n == *node; });
        if (it == Explored.end()) {
            Explored.push_back(node);
     }
    }

    void addToOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        openLists[heuristicIndex].insert(node);
    }

    void removeFromOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        std::shared_ptr<Node> existingNode = findNodeByPosition(openLists, heuristicIndex, node->position);
        if (existingNode) {
             openLists[heuristicIndex].erase(existingNode);
        }
    }


    std::shared_ptr<Node> findNodeByPosition(const std::vector<std::set<std::shared_ptr<Node>,NodeCompare>>& openLists, int heuristicIndex, const std::array<int, 3>& position) {
        auto& openList = openLists[heuristicIndex];
        for (const auto& node : openList) {
            if (node->position == position) {
                return node;
            }
        }
        return nullptr; 
    }

    void updateOpenList(int heuristicIndex, std::shared_ptr<Node> node) {
        std::shared_ptr<Node> existingNode = findNodeByPosition(openLists, heuristicIndex, node->position);
        if (existingNode) {
            if (existingNode->f_cost > node->f_cost) {
                removeFromOpenList(heuristicIndex, existingNode);
                addToOpenList(heuristicIndex, node);
            }
        } else {
            addToOpenList(heuristicIndex, node);
        }
    }


    void clearAllOpen() 
    {
        for (auto& openList : openLists) 
        {
            openList.clear();
        }
    }

    std::set<Resolution::Level> getUniqueResolutionLevels(const std::vector<std::set<Node>>& openLists, size_t index) //also usable for closeLists 
    {
        std::set<Resolution::Level> uniqueLevels;
        if (index < openLists.size()) {
            for (const auto& node : openLists[index]) {
                uniqueLevels.insert(node.res);
            }
    }
    return uniqueLevels;
    }   

    void addToCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            if (resLevel < static_cast<int>(closeList.size()))
            {
                closeList[resLevel].push_back(node);
            }
        }
    }

    bool isInCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) const {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            const auto& list = closeList[resLevel];
            return std::any_of(list.begin(), list.end(),
                            [node](const std::shared_ptr<Node>& n) { return *n == *node; });
        }
        return false;
    }

    std::shared_ptr<Node> whereInCloseList(const std::shared_ptr<Node>& node, Resolution::Level res) const {
        if (res != Resolution::Invalid) {
            int resLevel = static_cast<int>(res);
            const auto& list = closeList[resLevel];
            auto it = std::find_if(list.begin(), list.end(),
                            [node](const std::shared_ptr<Node>& n) { return *n == *node; });
            if (it != list.end())
            {
                return *it;
            }
        }
        return nullptr;
    }

    void clearAllClose() 
    {
        for (auto& list : closeList) 
        {
            list.clear();
        }
    }

    void addToINCONS(const std::shared_ptr<Node>& node) {
        auto it = std::find(INCONS.begin(), INCONS.end(), node);
        if (it == INCONS.end()) {
            INCONS.push_back(node);  
        }
    }

    void clearINCONS() {
        INCONS.clear(); 
    }

    void reintegrateINCONSIntoOpenList() 
    {
        for (const auto& node : INCONS) {
            updateOpenList(0, node);  
        }
        clearINCONS();  
    }

    double getTime()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
        //std::cout << "Milliseconds since epoch: " << milliseconds << " ms" << std::endl;
        return milliseconds;
    }



public:
    //std::vector<Node> Explored;
    AMRAstar(int initWeight1, int initWeight2, const std::string& path, 
        const std::array<int, 3>& taskStart, std::array<double, 4> startOri,
        const std::array<int, 3>& taskGoal, std::array<double, 4> goalOri,
        int resolutionScale, double timeLimit)
        : grid(path, resolutionScale), weight1(initWeight1), weight2(initWeight2), weight1_init(initWeight1), weight2_init(initWeight2), timeLimit(timeLimit) {
        
        manager.registerHeuristic("Euclidean", std::make_unique<EuclideanDistance>());
        heurs_map.emplace_back(Resolution::HIGH, manager.countHeuristics() - 1);

        manager.registerHeuristic("InadManhattan", std::make_unique<InadManhattanDistance>());
        heurs_map.emplace_back(Resolution::MID, manager.countHeuristics() - 1);

        manager.registerHeuristic("Manhattan", std::make_unique<ManhattanDistance>());
        heurs_map.emplace_back(Resolution::MID, manager.countHeuristics() - 1);

        openLists.resize(manager.countHeuristics());
        closeList.resize(4); // anchor,high,mid,low

        //double taskGoalGvalue = decodeGvalue(grid.getMapValue(taskGoal));
        totalDistance = manager.getHeuristic("Euclidean")->calculate(taskStart, taskGoal);
        std::cout<<"Euclidean_start: "<<totalDistance<<std::endl;
        start = std::make_shared<Node>(Node {taskStart,startOri, Resolution::HIGH, 0, totalDistance, weight1*totalDistance, nullptr});
        goal = std::make_shared<Node>(Node {taskGoal, goalOri, Resolution::HIGH, totalDistance, 0, totalDistance, nullptr});

        weight1Step = 0.1 * weight1;
        weight2step = 0.1 * weight2;

        INCONS.push_back(start);
    }

    double key(const Node& node, int i)
    {
        const Heuristic* heuristic = manager.getHeuristicByIndex(i);
        return node.g_cost +  weight1*heuristic->calculate(node.position, goal->position);
    }

    RotationQuaternion setOrientation(std::shared_ptr<Node> node, double distanceToGoal)
    {
        double ratio = distanceToGoal / totalDistance;
        double t;
        if (ratio <= 0.1) {
            return goal->orientation;
        } else if (ratio <= 0.3) {
            t = 0.7;
        } else if (ratio <= 0.5) {
            t = 0.5;
        } else if (ratio <= 0.7){
            t = 0.3;
        }else {
            t = 0.1;
        }
        return RotationQuaternion::slerp(node->orientation, goal->orientation, t);
        
    }
    


    void printOpen(int heuristicIndex) {
        if (heuristicIndex < static_cast<int>(openLists.size())) {
            std::cout << "Open List [" << heuristicIndex << "] Nodes:" << std::endl;
            for (const auto& node : openLists[heuristicIndex]) {
                std::cout << '(' << node->position[0] << ',' << node->position[1] << ',' << node->position[2] << ',' << node->f_cost << ')';
                if (node->parent) {  // check empty?
                    std::cout << " Parent: (" << node->parent->position[0] << ',' << node->parent->position[1] << ',' << node->parent->position[2] << ')';
                } else {
                    std::cout << " Parent: (null)";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "Invalid heuristic index." << std::endl;
        }
    }

    void printClose(Resolution::Level res) {
        int resLevel = static_cast<int>(res);
        if (resLevel < static_cast<int>(closeList.size())) {
            std::cout << "Close List [" << resLevel << "] Nodes:" << std::endl;
            for (const auto& node : closeList[resLevel]) {
                std::cout << '(' << node->position[0] << ',' << node->position[1] << ',' << node->position[2] << ',' << node->f_cost << ')';
                if (node->parent) {  // check empty
                    std::cout << " Parent: (" << node->parent->position[0] << ',' << node->parent->position[1] << ',' << node->parent->position[2] << ')';
                } else {
                    std::cout << " Parent: (null)";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "Invalid resolution level index." << std::endl;
        }
    }

    void displaySolution(){
        for (const auto& step : solutionPath) {
            std::cout << '(' << step.position[0] << ',' << step.position[1] << ',' << step.position[2] << ')';
            step.orientation.print();
        }
    }

    const std::vector<Node>& getSolution() const {
        return solutionPath;
    }


    void expand(std::shared_ptr<Node> node, int heuristicIndex) {
        Resolution::Level res = heurs_map.at(heuristicIndex).first;
        std::vector<std::pair<std::array<int, 3>, std::string>> successors;

        if (node->position == goal->position && node->orientation == goal->orientation)
            return;

        successors = grid.getSuccs(node->position, res);
        


        for (const auto& successorPosition : successors) {
            std::shared_ptr<Node> successor = std::make_shared<Node>(successorPosition.first, node->orientation, node->res, 0.0,0.0,0.0,nullptr);
            successor->parent = node;
            const Heuristic* heuristic = manager.getHeuristicByIndex(0);
            double h_cost_new = heuristic->calculate(successorPosition.first, goal->position);
            successor->h_cost = h_cost_new;
            successor->orientation = setOrientation(node,h_cost_new);
            double path_cost = heuristic->calculate(node->position, successorPosition.first);
            double g_cost_new = node->g_cost + path_cost + RotationQuaternion::distance(node->orientation,successor->orientation)*path_cost/3.14;
            successor->g_cost = g_cost_new;
            successor->f_cost = successor->g_cost + weight1 * successor->h_cost;

            if (res == Resolution::ANCHOR)
            {
                auto it = whereInCloseList(successor,res);
                if (it!= nullptr)
                {
                    if (it->g_cost <= successor->g_cost)
                    {
                        successor->g_cost = it->g_cost;
                        successor->parent = it->parent;
                        addToINCONS(successor);
                    }
                    addToINCONS(successor);
                }
            }
            else
            {
                updateOpenList(0,successor);          
                for (size_t j = 1; j < manager.countHeuristics(); ++j) 
                {
                    std::shared_ptr<Node> nodeCopy = successor;
                    Resolution::Level lres = heurs_map.at(j).first;
                    if (lres < nodeCopy->res)
                    {
                        continue;
                    }
                    if (!whereInCloseList(nodeCopy,lres))
                    {
                        double newKey = key(*nodeCopy, j);
                        if(newKey <= weight2*nodeCopy->f_cost)
                        {
                            nodeCopy->f_cost = newKey;
                            updateOpenList(j,nodeCopy);
                            //addToOpenList(j,successor);
                        }
                    }
                }
                addNodeToExplored(successor);
            }
        }
    }

    
    bool improvePath(const double& startTime) 
    {
        std::cout << "count" << std::endl;
        bool checkImprove = false;
        std::cout << "size of Heuristics" << manager.countHeuristics() << std::endl;

        for (size_t i = 0; i < manager.countHeuristics(); ++i) {
            while (!openLists[i].empty()) {
                //int i = iter;
                std::cout << "or_i: " <<i<< std::endl;
                if (i < manager.countHeuristics()-1){++i;}
                std::cout << "added_i: "<< i << std::endl;
                searchTime = getTime() - startTime;
                std::cout << "Elapsed Time: " << searchTime << std::endl;
                if (searchTime >= timeLimit) {
                    return false;
                }
                double fCheck = weight2 * (*openLists[0].begin())->f_cost;
                std::cout << "f_check: "<< fCheck << std::endl;
                if ((*openLists[i].begin())->f_cost > fCheck) {
                    i = 0;
                }
                std::cout << "i_after_check: "<< i << std::endl;
                std::shared_ptr<Node> x = *openLists[i].begin();
                openLists[i].erase(openLists[i].begin());

                std::cout << "x_now in "<<i<<": " << '(' << x->position[0] << ',' << x->position[1] << ',' << x->position[2] << ',' << x->f_cost << ')' << std::endl;
                if (x->position == goal->position && x->orientation== goal->orientation) {
                    std::cout << "beginToreconstruct" << std::endl;
                    reconstructPath(x);
                    checkImprove = true;
                    return true;
                }

                expand(x, i);  // Expand using smart pointers
                Resolution::Level res = heurs_map.at(i).first;
                addToCloseList(x, res);
                std::cout << "After expansion openList size: " << openLists[i].size() << std::endl;
                //std::cout << "open" << i << ':' << std::endl;
                //printOpen(i);
            }
        }
        return checkImprove;
    }
    



    void reconstructPath(const std::shared_ptr<Node>& node) {
        solutionPath.clear();
        for (std::shared_ptr<Node> current = node; current != nullptr; current = current->parent) {
            solutionPath.push_back(*current);
        }
        std::reverse(solutionPath.begin(), solutionPath.end());

        // Display the path
        for (const auto& step : solutionPath) {
            std::cout << '(' << step.position[0] << ',' << step.position[1] << ',' << step.position[2] << ')';
            step.orientation.print();
        }
    }


    void search()
    {
        startTime = getTime();
        searchTime = 0;
        //add map update!!!
        clearAllOpen();
        clearINCONS();
        clearAllClose();
        Explored.clear();
        //addToOpenList(0, start);
        addToINCONS(start);
        solutionPath.clear();

        while (weight1 >= 1 && weight2 >= 1) {
            std::cout << "weight1: " << weight1 << ", weight2: " << weight2 << std::endl;

            // Update nodes in INCONS
            for (const auto& node : INCONS) {
                updateOpenList(0, node);  
                //addToOpenList(0,node);
            }
            clearINCONS();

            // Update all open lists based on resolution
            for (size_t i = 0; i < openLists[0].size(); ++i) {
                auto& node = *std::next(openLists[0].begin(), i);
                for (size_t j = 1; j < manager.countHeuristics(); ++j) {
                    Resolution::Level res = heurs_map.at(j).first; //resolution of j
                    if (res >= node->res) {
                        std::shared_ptr<Node> nodeCopy = std::make_shared<Node>(*node);
                        nodeCopy->f_cost = key(*nodeCopy,j);
                        updateOpenList(j,nodeCopy);
                        //addToOpenList(j,nodeCopy);
                    }
                }
            }
            //printOpen(0);
            //printOpen(1);

            clearAllClose();
            if (searchTime > timeLimit){
                std::cout<<"Out of time !!!"<<std::endl;
                return;
            }

            if (improvePath(startTime)) {
                //reconstructPath(goal);
                std::cout << "Path found!" << std::endl;
                //return;
            }

            weight1 -= weight1Step;
            weight2 -= weight2step;
    }

    ThreeDMap Mysolution;
    for( auto const &solution: solutionPath)
    {
        Mysolution.addPath(solution.position,"sol");
    }
    Mysolution.saveToFile("MySolutionPath.map");
    ThreeDMap MyExplored;
    for( auto const &explored : Explored)
    {
        MyExplored.addPath(explored->position,"exp");
    }
    MyExplored.saveToFile("MyExploredNodes.map");
    }


     void reSearchPath(std::array<int, 3> startPosition, 
                   std::array<double, 4> startOri, 
                   std::array<int, 3> goalPosition, 
                   std::array<double, 4> goalOri)
    {
        totalDistance = manager.getHeuristic("Euclidean")->calculate(startPosition, goalPosition);
        weight1 = weight1_init;
        weight2 = weight2_init;
        weight1Step = 0.1 * weight1;
        weight2step = 0.1 * weight2;
        start = std::make_shared<Node>(Node {startPosition,startOri, Resolution::HIGH, 0, totalDistance, weight1*totalDistance, nullptr});
        goal = std::make_shared<Node>(Node {goalPosition, goalOri, Resolution::HIGH, totalDistance, 0, totalDistance, nullptr});
        search();
    }


};

}

#endif // AMRA_STAR_HPP