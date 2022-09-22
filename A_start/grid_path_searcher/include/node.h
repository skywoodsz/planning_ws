//
// Created by skywoodsz on 2022/9/21.
//

#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

// 右移20位
#define inf 1>>20

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    int id;
    Eigen::Vector3d coord; // corrdination in map
    Eigen::Vector3i dir;
    Eigen::Vector3i index;

    double gScore, fScore;
    GridNodePtr cameFrom; // parent node
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){
    id = 0;
    index = _index;
    coord = _coord;

    dir = Eigen::Vector3i::Zero();

    gScore = inf;
    fScore = inf;
    cameFrom = NULL;
    }
    GridNode(){};
    ~GridNode(){};
};

#endif //SRC_NODE_H
