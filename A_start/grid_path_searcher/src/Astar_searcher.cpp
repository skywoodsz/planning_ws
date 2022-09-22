//
// Created by skywoodsz on 2022/9/21.
//

#include <Astar_searcher.h>

/**
 * @brief 初始化地图
 *
 * @param _resolution
 * @param global_xyz_l：地图下限
 * @param global_xyz_u：地图上限
 * @param max_x_id
 * @param max_y_id
 * @param max_z_id
 */
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE]; // 总共的path点数
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t)); // 填充指定内容至内存

    // 三个一维数组组成一个三维数组
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}
/**
 * calculate h(n) use grid distance
 * @param node1
 * @param node2
 * @return
 */
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {

//    double euclidean = (node1 -> coord - node2 -> coord).norm() +
//                       static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000; // mit tie_breaker
//
//    return euclidean;

    Eigen::Vector3i start =node1->index;
    Eigen::Vector3i end =node2->index;

    int dx = abs(start(0) - end(0));
    int dy = abs(start(1) - end(1));
    int dz = abs(start(2) - end(2));


    // Dijkstra
    //    double hu = 0;

    // Euclidean
//        double hu = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));

    // Manhattan
    //    double hu = dx + dy+ dz;

    // Diagonal
    int dt = min({dx, dy, dz});
    double hu = sqrt(3) * dt;
    if(dt == dx)
    {
        hu += sqrt(2) * min({(dy - dt), (dz - dt)}) + abs(dy - dz);
    }
    else if(dt == dy)
    {
        hu += sqrt(2) * min({(dx - dt), (dz - dt)}) + abs(dz - dx);
    }
    else
    {
        hu += sqrt(2) * min({(dx - dt), (dy - dt)}) + abs(dx - dy);
    }

    hu = hu *( 1 + static_cast <double> (rand()) / static_cast <double> (RAND_MAX) / 1000);

    return hu;
}

/**
 * A start path planning
 * @param start_pt
 * @param end_pt
 */
void AstarPathFinder::AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {

    ros::Time time_1 = ros::Time::now();
    // get index of start_point and end_point
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // range position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    openSet.clear(); // open list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1; // open set
    startPtr->coord = start_pt;

    // 0. init the open set with start node
    openSet.insert(make_pair(startPtr->fScore, startPtr));

    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] = startPtr;
    double tentative_gScore;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    while(!openSet.empty())
    {
        // 1. Remove the node with lowest cost function from open set to closed set
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentPtr->id = -1; // closed set

        // 2. judge weather goal node
        if(currentPtr->index == goalIdx){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m",
                     (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution ); // index * resolution -> coord
            return;
        }

        // 3. get neighbor node and edge index cost
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        // 4. loop unexpanded neigbors
        for (int i = 0; i < (int)neighborPtrSets.size(); ++i) {
            // 4.1 judge if node is unexpanded
            //     neighborPtrSets[i]->id = 0 : unexpanded
            //     neighborPtrSets[i]->id = 1 : expanded, equal to this node is in open set
            //     neighborPtrSets[i]->id = -1 : expanded, equal to this node is in closed set

            neighborPtr = neighborPtrSets[i];
            // unexpanded & not in closed and open set
            // need set fScore
            if(neighborPtr->id == 0)
            {
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->id = 1;
                neighborPtr->cameFrom = currentPtr; // parent node
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

                continue;
            }
            // unexpanded & in open set
            // need update fScore
            else if(neighborPtr->id == 1)
            {
                auto open_element = openSet.begin();
                while(open_element->second != neighborPtr)
                    ++open_element;
                if(open_element->first > neighborPtr->fScore)
                {
                    open_element->second->fScore = neighborPtr->fScore;
                    open_element->second->cameFrom = currentPtr;
                }
                continue;
            }
            // expanded, equal to this node is in closed set
            else
            {
                continue; // nothing to do
            }

        }
    }

    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

/**
 * get neighbor node and edge cost
 * @param currentPtr
 * @param neighborPtrSets
 * @param edgeCostSets
 */
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets,
                                   std::vector<double> &edgeCostSets) {
    neighborPtrSets.clear();
    edgeCostSets.clear();

    Eigen::Vector3i current_idx = currentPtr->index;
    Eigen::Vector3d current_coord = currentPtr->coord;
//    Eigen::Vector3d current_coord = gridIndex2coord(current_idx);

    for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            for (int k = -1; k < 2; ++k) {
                if(i && j && k)
                    continue; // the node is currrent node

                    int s_x = current_idx[0] + i;
                    int s_y = current_idx[1] + j;
                    int s_z = current_idx[2] + k;

                    Eigen::Vector3i tmp_idx(s_x,s_y, s_z);

                    if(s_x > GLX_SIZE || s_y > GLY_SIZE || s_z > GLZ_SIZE || s_x < 0 || s_y < 0 || s_z <0)
                        continue; // out if the map range
                    if(isOccupied(tmp_idx))
                        continue; // node is occupied

                    Eigen::Vector3d tmp_pt = gridIndex2coord(tmp_idx);

                    neighborPtrSets.push_back(GridNodeMap[s_x][s_y][s_z]);
                    edgeCostSets.push_back((current_idx - tmp_idx).norm()); // index cost
//                    edgeCostSets.push_back((current_coord - tmp_pt).norm());
            }
        }
    }
}

/**
 * get parent node to build path
 * @return
 */
std::vector<Eigen::Vector3d> AstarPathFinder::getPath() {

    vector<Eigen::Vector3d> path;
    vector<GridNodePtr> gridPath;

    auto tmpPtr = terminatePtr;
    while((tmpPtr->cameFrom) != NULL)
    {
        gridPath.push_back(tmpPtr);
        tmpPtr = tmpPtr->cameFrom;
    }
    for (auto ptr : gridPath) {
        path.push_back(ptr->coord);
    }

    reverse(path.begin(), path.end());

    return path;
}

std::vector<Eigen::Vector3d> AstarPathFinder::getVisitedNodes() {

    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/**
 * reset the path
 * @param ptr
 */
void AstarPathFinder::resetGrid(GridNodePtr ptr) {
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids(){
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

/**
 * set obstacle with map coord
 * @param coord_x
 * @param coord_y
 * @param coord_z
 */
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z) {

    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
    return gridIndex2coord(coord2gridIndex(coord));
}

/**
 * @brief 由索引的出map坐标 原点为地图中心，因此存在0.5
 * idx ori = (gl_xl, gl_yl, gl_zl)
 * @param index
 * @return Vector3d
 */
Eigen::Vector3d AstarPathFinder::gridIndex2coord(const Eigen::Vector3i &index) {
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/**
 * to get idx with map coord
 * @param pt
 * @return
 */
Eigen::Vector3i AstarPathFinder::coord2gridIndex(const Eigen::Vector3d &pt) {
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
             (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}