#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>


enum EventType
{
    IN,
    IN_TOP,
    IN_BOTTOM,
    OUT,
    OUT_TOP,
    OUT_BOTTOM,
    INNER_IN,
    INNER_IN_TOP,
    INNER_IN_BOTTOM,
    INNER_OUT,
    INNER_OUT_TOP,
    INNER_OUT_BOTTOM,

    IN_EX,
    IN_TOP_EX,
    IN_BOTTOM_EX,
    OUT_EX,
    OUT_TOP_EX,
    OUT_BOTTOM_EX,
    INNER_IN_EX,
    INNER_IN_TOP_EX,
    INNER_IN_BOTTOM_EX,
    INNER_OUT_EX,
    INNER_OUT_TOP_EX,
    INNER_OUT_BOTTOM_EX,

    MIDDLE,
    CEILING,
    FLOOR,
    UNALLOCATED
};

enum VisualizationMode{PATH_MODE, ROBOT_MODE};

const int TOPLEFT = 0;
const int BOTTOMLEFT = 1;
const int BOTTOMRIGHT = 2;
const int TOPRIGHT = 3;

const int palette_colors = 1530;

class Point2D
{
public:
    Point2D()
    {
        x = INT_MAX;
        y = INT_MAX;
    }
    Point2D(int x_pos, int y_pos)
    {
        x = x_pos;
        y = y_pos;
    }
    Point2D(const Point2D& point)
    {
        x = point.x;
        y = point.y;
    }
    int x;
    int y;
};

/** 多边形顶点按照逆时针旋转排序 **/
typedef std::vector<Point2D> Polygon;
typedef std::vector<Polygon> PolygonList;
typedef std::deque<Point2D> Edge;

class Event
{
public:
    Event(int obstacle_idx, int x_pos, int y_pos, EventType type=UNALLOCATED)
    {
        obstacle_index = obstacle_idx;
        x = x_pos;
        y = y_pos;
        event_type = type;
        original_index_in_slice = INT_MAX;
        isUsed = false;
    }

    int x;
    int y;
    int original_index_in_slice;
    int obstacle_index;
    EventType event_type;

    bool isUsed;
};

class CellNode
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
        cellIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;
    Edge ceiling;
    Edge floor;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};

bool operator<(const Point2D& p1, const Point2D& p2)
{
    return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
}

bool operator<(const Event& e1, const Event& e2)
{
    return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y) || (e1.x == e2.x && e1.y == e2.y && e1.obstacle_index < e2.obstacle_index));
}

bool operator==(const Point2D& p1, const Point2D& p2)
{
    return (p1.x==p2.x && p1.y==p2.y);
}

bool operator!=(const Point2D& p1, const Point2D& p2)
{
    return !(p1==p2);
}

int WrappedIndex(int index, int list_length)
{
    int wrapped_index = (index%list_length+list_length)%list_length;
    return wrapped_index;
}

/** 深度优先搜索遍历邻接图 **/
void WalkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path)
{
    if(!cell_graph[cell_index].isVisited)
    {
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);

//    for debugging
//    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;
//

    CellNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++)
    {
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited)
        {
            break;
        }
    }

    if(!neighbor.isVisited) // unvisited neighbor found
    {
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        WalkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else if(unvisited_counter == 0)
        {
            return;
        }
        else
        {
            WalkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}

std::deque<CellNode> GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index)
{
    std::deque<CellNode> visitting_path;

    if(cell_graph.size()==1)
    {
        visitting_path.emplace_back(cell_graph.front());
    }
    else
    {
        int unvisited_counter = cell_graph.size();
        WalkThroughGraph(cell_graph, first_cell_index, unvisited_counter, visitting_path);
        std::reverse(visitting_path.begin(), visitting_path.end());
    }

    return visitting_path;
}

std::vector<Point2D> ComputeCellCornerPoints(const CellNode& cell)
{

    Point2D topleft = cell.ceiling.front();
    Point2D bottomleft = cell.floor.front();
    Point2D bottomright = cell.floor.back();
    Point2D topright = cell.ceiling.back();

    // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
    std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright, topright};

    return corner_points;
}

std::vector<int> DetermineCellIndex(std::vector<CellNode>& cell_graph, const Point2D& point)
{
    std::vector<int> cell_index;

    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].ceiling.size(); j++)
        {
            if(point.x ==  cell_graph[i].ceiling[j].x && point.y >= cell_graph[i].ceiling[j].y && point.y <= cell_graph[i].floor[j].y)
            {
                cell_index.emplace_back(int(i));
            }
        }

    }
    return cell_index;
}

std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius)
{
    int delta, increment;

    std::deque<Point2D> path;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

    std::vector<Point2D> ceiling, floor;
    ceiling.assign(cell.ceiling.begin(), cell.ceiling.end());
    floor.assign(cell.floor.begin(), cell.floor.end());

    if(cell_graph[cell.cellIndex].isCleaned)
    {
        if(corner_indicator == TOPLEFT)
        {
            path.emplace_back(corner_points[TOPLEFT]);
        }
        if(corner_indicator == TOPRIGHT)
        {
            path.emplace_back(corner_points[TOPRIGHT]);
        }
        if(corner_indicator == BOTTOMLEFT)
        {
            path.emplace_back(corner_points[BOTTOMLEFT]);
        }
        if(corner_indicator == BOTTOMRIGHT)
        {
            path.emplace_back(corner_points[BOTTOMRIGHT]);
        }
    }
    else
    {
        if(corner_indicator == TOPLEFT)
        {
            int x, y, y_start, y_end;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i = i + (robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))
                    {
                        delta = floor[i+1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从左往右
                            if( x+j >= floor.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            //提前转
                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
                               &&(j<=robot_radius)
                               &&(j+1<=robot_radius))
                            {
                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
                                }
                            }
                            //滞后转
                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                                    &&(j+1<=robot_radius)
                                    &&(j<=robot_radius))
                            {
                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));

                                delta = floor[i+(j+1)].y-floor[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i+(j)]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
                    {
                        delta = ceiling[i+1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着ceiling从左往右
                            if(x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            // 提前转
                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
                               &&(j+1 <= robot_radius)
                               &&(j <= robot_radius))
                            {
                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
                                }
                            }
                            // 滞后转
                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                                    &&(j<=robot_radius)
                                    &&(j+1<=robot_radius))
                            {
                                path.emplace_back(ceiling[i+(j)]);

                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i+j]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == TOPRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
                    {
                        delta = floor[i-1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= floor.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            //提前转
                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
                               &&(j<=robot_radius)
                               &&(j+1<=robot_radius))
                            {
                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
                                }
                            }
                            //滞后转
                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                                    &&(j+1<=robot_radius)
                                    &&(j<=robot_radius))
                            {
                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));

                                delta = floor[i-(j+1)].y-floor[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i-(j)]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
                    {
                        delta = ceiling[i-1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着ceiling从右往左
                            if( x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
                               &&(j+1 <= robot_radius)
                               &&(j <= robot_radius))
                            {
                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
                                }
                            }
                            // 滞后转
                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                                    &&(j<=robot_radius)
                                    &&(j+1<=robot_radius))
                            {
                                path.emplace_back(ceiling[i-(j)]);

                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i-j]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = 0; i < ceiling.size(); i=i+(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i+1].y-ceiling[i].y)>=2)&&(i+1<ceiling.size()))
                    {
                        delta = ceiling[i+1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着ceiling从左往右
                            if(x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i+(j+1)].y-ceiling[i+(j)].y>=2)
                               &&(j+1 <= robot_radius)
                               &&(j <= robot_radius))
                            {
                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+j].x, ceiling[i+j].y+increment*(k)));
                                }
                            }
                                // 滞后转
                            else if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                                    &&(j<=robot_radius)
                                    &&(j+1<=robot_radius))
                            {
                                path.emplace_back(ceiling[i+(j)]);

                                delta = ceiling[i+(j+1)].y-ceiling[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i+(j+1)].x, ceiling[i+(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i+j]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i+1].y-floor[i].y)>=2)&&(i+1<floor.size()))
                    {
                        delta = floor[i+1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从左往右
                            if(x+j >= floor.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            //提前转
                            else if((floor[i+(j)].y-floor[i+(j+1)].y>=2)
                               &&(j<=robot_radius)
                               &&(j+1<=robot_radius))
                            {
                                delta = floor[i+(j+1)].y-floor[i+(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y+increment*(k)));
                                }
                            }
                                //滞后转
                            else if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                                    &&(j+1<=robot_radius)
                                    &&(j<=robot_radius))
                            {
                                path.emplace_back(Point2D(floor[i+(j)].x, floor[i+(j)].y));

                                delta = floor[i+(j+1)].y-floor[i+(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i+(j+1)].x, cell.floor[i+(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i+(j)]);
                            }
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1; i >= 0; i=i-(robot_radius+1))
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y;
                    y_end   = ceiling[i].y;

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(ceiling[i-1].y-ceiling[i].y)>=2)&&(i-1>=0))
                    {
                        delta = ceiling[i-1].y-ceiling[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着ceiling从右往左
                            if(x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            // 提前转
                            else if((ceiling[i-(j+1)].y-ceiling[i-(j)].y>=2)
                               &&(j+1 <= robot_radius)
                               &&(j <= robot_radius))
                            {
                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-j].x, ceiling[i-j].y+increment*(k)));
                                }
                            }
                                // 滞后转
                            else if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                                    &&(j<=robot_radius)
                                    &&(j+1<=robot_radius))
                            {
                                path.emplace_back(ceiling[i-(j)]);

                                delta = ceiling[i-(j+1)].y-ceiling[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(ceiling[i-(j+1)].x, ceiling[i-(j+1)].y+abs(delta)+increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(ceiling[i-j]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y;
                    y_end   = floor[i].y;

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if((std::abs(floor[i-1].y-floor[i].y)>=2)&&(i-1>=0))
                    {
                        delta = floor[i-1].y-floor[i].y;
                        increment = delta/abs(delta);
                        for(int k = 1; k <= abs(delta); k++)
                        {
                            path.emplace_back(Point2D(floor[i].x, floor[i].y+increment*(k)));
                        }
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= floor.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            //提前转
                            else if((floor[i-(j)].y-floor[i-(j+1)].y>=2)
                               &&(j<=robot_radius)
                               &&(j+1<=robot_radius))
                            {
                                delta = floor[i-(j+1)].y-floor[i-(j)].y;
                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y+increment*(k)));
                                }
                            }
                                //滞后转
                            else if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                                    &&(j+1<=robot_radius)
                                    &&(j<=robot_radius))
                            {
                                path.emplace_back(Point2D(floor[i-(j)].x, floor[i-(j)].y));

                                delta = floor[i-(j+1)].y-floor[i-(j)].y;

                                increment = delta/abs(delta);
                                for(int k = 0; k <= abs(delta); k++)
                                {
                                    path.emplace_back(Point2D(floor[i-(j+1)].x, cell.floor[i-(j+1)].y-abs(delta) +increment*(k)));
                                }
                            }
                            else
                            {
                                path.emplace_back(floor[i-(j)]);
                            }

                        }
                    }

                    reverse = !reverse;
                }
            }
        }
    }

    return path;
}

std::vector<Event> InitializeEventList(const Polygon& polygon, int polygon_index)
{
    std::vector<Event> event_list;

    for(const auto& point : polygon)
    {
        event_list.emplace_back(Event(polygon_index, point.x, point.y));
    }

    return event_list;
}

void AllocateObstacleEventType(const cv::Mat& map, std::vector<Event>& event_list)
{
    int index_offset;
    std::deque<int> in_out_index_list; // 只存放各种in和out的index

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN;
            in_out_index_list.emplace_back(i);
        }
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;
    int temp_index;

    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_OUT;
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM;
            }

        }

        if(event_list[in_out_index].event_type == IN)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_IN;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_IN_TOP;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==IN
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
}

void AllocateWallEventType(const cv::Mat& map, std::vector<Event>& event_list)
{
    int index_offset;
    std::deque<int> in_out_index_list; // 只存放各种in和out的index

    int N = event_list.size();

    // determine in and out and middle
    for(int i = 0; i < N; i++)
    {
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = IN_EX;
            in_out_index_list.emplace_back(i);
        }
        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x < event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x < event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x < event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = IN_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = MIDDLE;
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x)
        {
            event_list[i].event_type = OUT_EX;
            in_out_index_list.emplace_back(i);
        }


        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y < event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y < event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y < event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_TOP_EX;
                in_out_index_list.emplace_back(i);
            }
        }

        if(event_list[i].x == event_list[((i-1)%N+N)%N].x && event_list[i].x > event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i-1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i-index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i-index_offset)%N+N)%N].x && event_list[i].y > event_list[((i-index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }


        if(event_list[i].x > event_list[((i-1)%N+N)%N].x && event_list[i].x == event_list[((i+1)%N+N)%N].x && event_list[i].y > event_list[((i+1)%N+N)%N].y)
        {
            index_offset = 2;
            while(event_list[i].x == event_list[((i+index_offset)%N+N)%N].x)
            {
                index_offset++;
            }
            if(event_list[i].x > event_list[((i+index_offset)%N+N)%N].x && event_list[i].y > event_list[((i+index_offset)%N+N)%N].y)
            {
                event_list[i].event_type = OUT_BOTTOM_EX;
                in_out_index_list.emplace_back(i);
            }
        }
    }

    // determine inner
    Point2D neighbor_point;
    int temp_index;
    for(auto in_out_index : in_out_index_list)
    {
        if(event_list[in_out_index].event_type == OUT_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == OUT_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x+1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x < map.cols)
            {
                event_list[in_out_index].event_type = INNER_OUT_BOTTOM_EX;
            }

        }

        if(event_list[in_out_index].event_type == IN_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_EX;
            }
        }


        if(event_list[in_out_index].event_type == IN_TOP_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_TOP_EX;
            }
        }

        if(event_list[in_out_index].event_type == IN_BOTTOM_EX)
        {
            neighbor_point = Point2D(event_list[in_out_index].x-1, event_list[in_out_index].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0,0,0) && neighbor_point.x>=0)
            {
                event_list[in_out_index].event_type = INNER_IN_BOTTOM_EX;
            }
        }
    }

    // determine floor and ceiling
    std::deque<int> ceiling_floor_index_list;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(
                (event_list[in_out_index_list[0]].event_type==OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==IN_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = CEILING;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        if(
                (event_list[in_out_index_list[0]].event_type==IN_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP_EX
                 ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM_EX)
                &&
                (event_list[in_out_index_list[1]].event_type==OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP_EX
                 ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM_EX)
                )
        {
            if(in_out_index_list[0] < in_out_index_list[1])
            {
                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
            }
            else
            {
                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
                {
                    if(event_list[j].event_type != MIDDLE)
                    {
                        event_list[j].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(j);
                    }
                }
                for(int k = 0; k < in_out_index_list[1]; k++)
                {
                    if(event_list[k].event_type != MIDDLE)
                    {
                        event_list[k].event_type = FLOOR;
                        ceiling_floor_index_list.emplace_back(k);
                    }
                }
            }
        }

        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }


    // filter ceiling and floor
    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
    {
        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
           && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
        {
            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
            {
                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
            }
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
       && event_list[ceiling_floor_index_list.front()].event_type==CEILING
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
       && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
    {
        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
        {
            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
        }
        else
        {
            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
        }
    }
}

std::vector<Event> GenerateObstacleEventList(const cv::Mat& map, const PolygonList& polygons)
{
    std::vector<Event> event_list;
    std::vector<Event> event_sublist;

    for(int i = 0; i < polygons.size(); i++)
    {
        event_sublist = InitializeEventList(polygons[i], i);
        AllocateObstacleEventType(map, event_sublist);
        event_list.insert(event_list.end(), event_sublist.begin(), event_sublist.end());
        event_sublist.clear();
    }

    std::sort(event_list.begin(), event_list.end());

    return event_list;
}

std::vector<Event> GenerateWallEventList(const cv::Mat& map, const Polygon& external_contour)
{
    std::vector<Event> event_list;

    event_list = InitializeEventList(external_contour, INT_MAX);
    AllocateWallEventType(map, event_list);
    std::sort(event_list.begin(), event_list.end());

    return event_list;
}

std::deque<std::deque<Event>> SliceListGenerator(const std::vector<Event>& wall_event_list, const std::vector<Event>& obstacle_event_list)
{
    std::vector<Event> event_list;
    event_list.insert(event_list.end(), obstacle_event_list.begin(), obstacle_event_list.end());
    event_list.insert(event_list.end(), wall_event_list.begin(), wall_event_list.end());
    std::sort(event_list.begin(), event_list.end());

    std::deque<std::deque<Event>> slice_list;
    std::deque<Event> slice;
    int x = event_list.front().x;

    for(auto event : event_list)
    {
        if(event.x != x)
        {
            slice_list.emplace_back(slice);

            x = event.x;
            slice.clear();
            slice.emplace_back(event);
        }
        else
        {
            slice.emplace_back(event);
        }
    }
    slice_list.emplace_back(slice);

    return slice_list;
}

void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false)
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
    }
    else
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());

    }
}

void ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite = false)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(c);
    new_cell.floor.emplace_back(f);

    if(!rewrite)
    {
        int new_cell_idx = cell_graph.size();
        new_cell.cellIndex = new_cell_idx;

        cell_graph.emplace_back(new_cell);


        cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
        cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

        cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);
    }
    else
    {
        cell_graph[top_cell_idx].ceiling.assign(new_cell.ceiling.begin(), new_cell.ceiling.end());
        cell_graph[top_cell_idx].floor.assign(new_cell.floor.begin(), new_cell.floor.end());

        cell_graph[top_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);
        cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    }

}

void ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point)
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}

void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false)
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in_top);

    bottom_cell.ceiling.emplace_back(in_bottom);
    bottom_cell.floor.emplace_back(f);


    if(!rewrite)
    {
        int top_cell_index = cell_graph.size();
        int bottom_cell_index = cell_graph.size() + 1;

        top_cell.cellIndex = top_cell_index;
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(top_cell);
        cell_graph.emplace_back(bottom_cell);


        cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
        cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
    }
    else
    {
        cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
        cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

        int bottom_cell_index = cell_graph.size();
        bottom_cell.cellIndex = bottom_cell_index;
        cell_graph.emplace_back(bottom_cell);

        cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
        cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());
    }

}

void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in);
    new_cell.floor.emplace_back(inner_in);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void ExecuteInnerOpenOperation(std::vector<CellNode>& cell_graph, Point2D inner_in_top, Point2D inner_in_bottom)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in_top);
    new_cell.floor.emplace_back(inner_in_bottom);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
}

void ExecuteInnerCloseOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
}

void DrawCells(cv::Mat& map, const CellNode& cell, cv::Scalar color=cv::Scalar(100, 100, 100))
{
    std::cout<<"cell "<<cell.cellIndex<<": "<<std::endl;
    std::cout<<"cell's ceiling points: "<<cell.ceiling.size()<<std::endl;
    std::cout<<"cell's floor points: "<<cell.floor.size()<<std::endl;

    for(const auto& ceiling_point : cell.ceiling)
    {
        map.at<cv::Vec3b>(ceiling_point.y, ceiling_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    for(const auto& floor_point : cell.floor)
    {
        map.at<cv::Vec3b>(floor_point.y, floor_point.x) = cv::Vec3b(uchar(color[0]), uchar(color[1]), uchar(color[2]));
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), color);
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), color);
}

int CountCells(const std::deque<Event>& slice, int curr_idx)
{
    int cell_num = 0;
    for(int i = 0; i < curr_idx; i++)
    {
        if(
              (slice[i].event_type==IN)
           || (slice[i].event_type==IN_TOP)
           || (slice[i].event_type==INNER_IN)
           || (slice[i].event_type==INNER_IN_BOTTOM)
           || (slice[i].event_type==FLOOR)
           || (slice[i].event_type==IN_BOTTOM_EX)
           || (slice[i].event_type==INNER_IN_EX)
           || (slice[i].event_type==INNER_IN_TOP_EX)
          )
        {
            cell_num++;
        }
    }
    return cell_num;
}

std::deque<Event> FilterSlice(const std::deque<Event>& slice)
{
    std::deque<Event> filtered_slice;

    for(auto event : slice)
    {
        if(event.event_type!=MIDDLE && event.event_type!=UNALLOCATED)
        {
            filtered_slice.emplace_back(event);
        }
    }
    return filtered_slice;
}

void ExecuteCellDecomposition(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, const std::deque<std::deque<Event>>& slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    Point2D c, f;
    int c_index = INT_MAX, f_index = INT_MAX;
    int min_dist = INT_MAX;

    int event_y = INT_MAX;

    bool rewrite = false;

    std::vector<int> sub_cell_index_slices;
    std::deque<Event> curr_slice;

    int cell_counter = 0;

    for(const auto& raw_slice : slice_list)
    {
        curr_slice = FilterSlice(raw_slice);

        original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());

        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == INNER_IN_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                                          Point2D(curr_slice[j].x, curr_slice[j].y),
                                                          c,
                                                          f,
                                                          rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == INNER_OUT_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                           c,
                                                           f,
                                                           rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                                          Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                                          Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                                          c,
                                                          f,
                                                          rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                           c,
                                                           f,
                                                           rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == IN_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {
                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                    }

                }
                else
                {
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                    cell_index_slice.emplace_back(int(cell_graph.size()-1));
                    curr_slice[j].isUsed = true;
                }

            }

            if(curr_slice[j].event_type == IN_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                if(!cell_index_slice.empty())
                {
                    for(int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {

                            ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                         Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                            curr_slice[j-1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                    if(event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                     Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                    if(event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                     Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                }
                else
                {
                    ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                 Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                    cell_index_slice.emplace_back(int(cell_graph.size()-1));

                    curr_slice[j-1].isUsed = true;
                    curr_slice[j].isUsed = true;
                }

            }


            if(curr_slice[j].event_type == OUT_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == OUT_BOTTOM_EX)
            {
                event_y = curr_slice[j].y;

                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }


        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == IN)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end(); // 若为true，则覆盖

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y-cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j].x, curr_slice[j].y),
                                             c,
                                             f,
                                             rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }
            if(curr_slice[j].event_type == OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                            cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }


                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(cell_graph, curr_cell_idx,
                                             Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                             c,
                                             f,
                                             rewrite);


                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                            cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                    sub_cell_index_slices.end());
                        }
                        else
                        {
                            cell_index_slice.insert(cell_index_slice.begin()+k+1, int(cell_graph.size()-1));
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end();

                        min_dist = INT_MAX;
                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k-1]].ceiling.back().y);
                                c_index = m;
                                c = Point2D(curr_slice[m].x, curr_slice[m].y);
                            }
                        }
                        curr_slice[c_index].isUsed = true;

                        min_dist = INT_MAX;
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y)<min_dist)
                            {
                                min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                f_index = n;
                                f = Point2D(curr_slice[n].x, curr_slice[n].y);
                            }
                        }
                        curr_slice[f_index].isUsed = true;

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                              c,
                                              f,
                                              rewrite);

                        if(!rewrite)
                        {
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.erase(cell_index_slice.begin()+k-1);
                            cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        }
                        else
                        {
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                        }

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_IN)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_IN_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(cell_graph, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));    // inner_in_bottom

                        cell_index_slice.insert(cell_index_slice.begin()+k, int(cell_graph.size()-1));

                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;

                        break;
                    }
                }
            }


            if(curr_slice[j].event_type == INNER_OUT)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

            if(curr_slice[j].event_type == INNER_OUT_BOTTOM)
            {
                event_y = curr_slice[j].y;
                for(int k = 0; k < cell_index_slice.size(); k++)
                {
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
                        cell_index_slice.erase(cell_index_slice.begin()+k);
                        curr_slice[j-1].isUsed = true;
                        curr_slice[j].isUsed = true;
                        break;
                    }
                }
            }

        }

        for(int j = 0; j < curr_slice.size(); j++)
        {
            if(curr_slice[j].event_type == CEILING)
            {
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteCeilOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
            if(curr_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteFloorOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
        }
    }
}

Point2D FindNextEntrance(const Point2D& curr_point, const CellNode& next_cell, int& corner_indicator)
{
    Point2D next_entrance;

    int front_x = next_cell.ceiling.front().x;
    int back_x = next_cell.ceiling.back().x;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(next_cell);

    if(abs(curr_point.x - front_x) < abs(curr_point.x - back_x))
    {
        if(abs(curr_point.y - next_cell.ceiling.front().y)<abs(curr_point.y - next_cell.floor.front().y))
        {
            next_entrance = corner_points[TOPLEFT];
            corner_indicator = TOPLEFT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMLEFT];
            corner_indicator = BOTTOMLEFT;
        }
    }
    else
    {
        if(abs(curr_point.y - next_cell.ceiling.back().y)<abs(curr_point.y - next_cell.floor.back().y))
        {
            next_entrance = corner_points[TOPRIGHT];
            corner_indicator = TOPRIGHT;
        }
        else
        {
            next_entrance = corner_points[BOTTOMRIGHT];
            corner_indicator = BOTTOMRIGHT;
        }
    }

    return next_entrance;
}

std::deque<Point2D> WalkInsideCell(CellNode cell, const Point2D& start, const Point2D& end)
{
    std::deque<Point2D> inner_path = {start};

    int start_ceiling_index_offset = start.x - cell.ceiling.front().x;
    int first_ceiling_delta_y = cell.ceiling[start_ceiling_index_offset].y - start.y;
    int end_ceiling_index_offset = end.x - cell.ceiling.front().x;
    int second_ceiling_delta_y = end.y - cell.ceiling[end_ceiling_index_offset].y;

    int start_floor_index_offset = start.x - cell.floor.front().x;
    int first_floor_delta_y = cell.floor[start_floor_index_offset].y - start.y;
    int end_floor_index_offset = end.x - cell.floor.front().x;
    int second_floor_delta_y = end.y - cell.floor[end_floor_index_offset].y;

    if((abs(first_ceiling_delta_y)+abs(second_ceiling_delta_y)) < (abs(first_floor_delta_y)+abs(second_floor_delta_y))) //to ceiling
    {
        int first_increment_y = 0;
        if(first_ceiling_delta_y != 0)
        {
            first_increment_y = first_ceiling_delta_y / abs(first_ceiling_delta_y);

            for(int i = 1; i <= abs(first_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.ceiling[end_ceiling_index_offset].x - cell.ceiling[start_ceiling_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            // 提前转
            if((cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y>=2)
               &&(i+1 <= abs(delta_x))
               &&(i <= abs(delta_x)))
            {
                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*i].x, cell.ceiling[start_ceiling_index_offset+increment_x*i].y+increment*(j)));
                }
            }
            // 滞后转
            else if((cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y>=2)
                     &&(i<=abs(delta_x))
                     &&(i+1<=abs(delta_x)))
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+increment_x*(i)]);

                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y+abs(delta)+increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset+(increment_x*i)]);
            }
        }

        int second_increment_y = 0;
        if(second_ceiling_delta_y!=0)
        {
            second_increment_y = second_ceiling_delta_y/abs(second_ceiling_delta_y);

            for(int i = 1; i <= abs(second_ceiling_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.ceiling[end_ceiling_index_offset].x, cell.ceiling[end_ceiling_index_offset].y+(second_increment_y*i)));
            }
        }

    }
    else // to floor
    {
        int first_increment_y = 0;
        if(first_floor_delta_y != 0)
        {
            first_increment_y = first_floor_delta_y / abs(first_floor_delta_y);

            for(int i = 1; i <= abs(first_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
            }
        }

        int delta_x = cell.floor[end_floor_index_offset].x - cell.floor[start_floor_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 0; i < abs(delta_x); i++)
        {
            //提前转
            if((cell.floor[start_floor_index_offset+increment_x*(i)].y-cell.floor[start_floor_index_offset+increment_x*(i+1)].y>=2)
               &&(i<=abs(delta_x))
               &&(i+1<=abs(delta_x)))
            {
                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y+increment*(j)));
                }
            }
            //滞后转
            else if((cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y>=2)
                    &&(i+1<=abs(delta_x))
                    &&(i<=abs(delta_x)))
            {
                inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y));

                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;

                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+1)].x, cell.floor[start_floor_index_offset+increment_x*(i+1)].y-abs(delta) +increment*(k)));
                }
            }
            else
            {
                inner_path.emplace_back(cell.floor[start_floor_index_offset+(increment_x*i)]);
            }

        }

        int second_increment_y = 0;
        if(second_floor_delta_y!=0)
        {
            second_increment_y = second_floor_delta_y/abs(second_floor_delta_y);

            for(int i = 1; i <= abs(second_floor_delta_y); i++)
            {
                inner_path.emplace_back(Point2D(cell.floor[end_floor_index_offset].x, cell.floor[end_floor_index_offset].y+(second_increment_y*i)));
            }
        }
    }
    return inner_path;
}

std::deque<std::deque<Point2D>> FindLinkingPath(const Point2D& curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, const CellNode& next_cell)
{
    std::deque<std::deque<Point2D>> path;
    std::deque<Point2D> path_in_curr_cell;
    std::deque<Point2D> path_in_next_cell;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator);
    path_in_curr_cell = WalkInsideCell(curr_cell, curr_exit, exit);

    next_entrance = FindNextEntrance(exit, next_cell, corner_indicator);

    int delta_x = next_entrance.x - exit.x;
    int delta_y = next_entrance.y - exit.y;

    int increment_x = 0;
    int increment_y = 0;

    if (delta_x != 0) {
        increment_x = delta_x / std::abs(delta_x);
    }
    if (delta_y != 0) {
        increment_y = delta_y / std::abs(delta_y);
    }

    int upper_bound = INT_MIN;
    int lower_bound = INT_MAX;

    if (exit.x >= curr_cell.ceiling.back().x)
    {
        upper_bound = curr_cell.ceiling.back().y;
        lower_bound = curr_cell.floor.back().y;
    }
    if (exit.x <= curr_cell.ceiling.front().x)
    {
        upper_bound = curr_cell.ceiling.front().y;
        lower_bound = curr_cell.floor.front().y;
    }

    if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound))
    {
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_curr_cell.emplace_back(Point2D(exit.x, y));
        }
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, next_entrance.y));
        }
    }
    else
    {
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path_in_curr_cell.emplace_back(Point2D(x, exit.y));
        }
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path_in_next_cell.emplace_back(Point2D(next_entrance.x, y));
        }
    }

    path = {path_in_curr_cell, path_in_next_cell};

    return path;
}

std::deque<Point2D> WalkCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, const Point2D& start, const Point2D& end, int robot_radius)
{
    std::deque<Point2D> overall_path;
    std::deque<Point2D> sub_path;

    std::deque<std::deque<Point2D>> link_path;

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(auto cell : cells)
    {
        cell.isCleaned = true;
    }

    Point2D curr_exit, next_entrance;
    int curr_corner_indicator, next_corner_indicator;

    next_entrance = FindNextEntrance(start, cells[cell_path[1]], next_corner_indicator);
    curr_exit = FindNextEntrance(next_entrance, cells[cell_path[0]], curr_corner_indicator);
    sub_path = WalkInsideCell(cells[cell_path[0]], start, curr_exit);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[0]], cells[cell_path[1]]);
    sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
    sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_corner_indicator = next_corner_indicator;


    for(int i = 1; i < cell_path.size()-1; i++)
    {
        sub_path = GetBoustrophedonPath(cell_graph, cells[cell_path[i]], curr_corner_indicator, robot_radius);
        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_exit = overall_path.back();
        next_entrance = FindNextEntrance(curr_exit, cells[cell_path[i+1]], next_corner_indicator);

        link_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[i]], cells[cell_path[i+1]]);
        sub_path.insert(sub_path.end(), link_path.front().begin(), link_path.front().end());
        sub_path.insert(sub_path.end(), link_path.back().begin(), link_path.back().end());


        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_corner_indicator = next_corner_indicator;
    }

    sub_path = WalkInsideCell(cells[cell_path.back()], next_entrance, end);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    return overall_path;
}

/** 广度优先搜索 **/
std::deque<int> FindShortestPath(std::vector<CellNode>& cell_graph, const Point2D& start, const Point2D& end)
{
    int start_cell_index = DetermineCellIndex(cell_graph, start).front();
    int end_cell_index = DetermineCellIndex(cell_graph, end).front();

    std::deque<int> cell_path = {end_cell_index};

    if(start_cell_index == end_cell_index)
    {
        return cell_path;
    }

    if(start_cell_index == end_cell_index)
    {
        cell_path.emplace_back(start_cell_index);
        return cell_path;
    }

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(auto cell : cells)
    {
        cell.isVisited = false;
        cell.isCleaned = false;
        cell.parentIndex = INT_MAX;
    }

    std::deque<int> search_queue = {start_cell_index};

    CellNode curr_cell;

    while(!search_queue.empty())
    {
        curr_cell = cells[search_queue.front()];

        cells[search_queue.front()].isVisited = true;
        search_queue.pop_front();

        for(int i = 0; i < curr_cell.neighbor_indices.size(); i++)
        {
            if(curr_cell.neighbor_indices[i] == end_cell_index)
            {
                cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
                search_queue.clear();
                break;
            }
            else if(!cells[curr_cell.neighbor_indices[i]].isVisited)
            {
                cells[curr_cell.neighbor_indices[i]].isVisited = true;
                cells[curr_cell.neighbor_indices[i]].parentIndex = curr_cell.cellIndex;
                search_queue.emplace_back(curr_cell.neighbor_indices[i]);
            }
        }

    }

    curr_cell = cells[end_cell_index];
    int prev_cell_index;

    while(curr_cell.parentIndex != INT_MAX)
    {
        prev_cell_index = curr_cell.parentIndex;
        cell_path.emplace_front(prev_cell_index);
        curr_cell = cells[prev_cell_index];
    }

    return cell_path;
}

void InitializeColorMap(std::deque<cv::Scalar>& JetColorMap, int repeat_times)
{
    for(int i = 0; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, i, 255));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(0, 255, i));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 255, 0));
        }
    }

    for(int i = 254; i >= 0; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, i, 0));
        }
    }

    for(int i = 1; i <= 255; i++)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(255, 0, i));
        }
    }

    for(int i = 254; i >= 1; i--)
    {
        for(int j = 0; j < repeat_times; j++)
        {
            JetColorMap.emplace_back(cv::Scalar(i, 0, 255));
        }
    }
}

void UpdateColorMap(std::deque<cv::Scalar>& JetColorMap)
{
    cv::Scalar color = JetColorMap.front();
    JetColorMap.pop_front();
    JetColorMap.emplace_back(color);
}





int ComputeRobotRadius(const double& meters_per_pix, const double& robot_size_in_meters)
{
    int robot_radius = int(robot_size_in_meters / meters_per_pix);
    return robot_radius;
}

cv::Mat1b ReadMap(const std::string& map_file_path)
{
    cv::Mat1b original_map = cv::imread(map_file_path, CV_8U);
    return original_map;
}

cv::Mat1b PreprocessMap(const cv::Mat1b& original_map)
{
    cv::Mat1b map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY);
    return map;
}

/** 默认是空闲区域为白色，障碍物为黑色 **/
void ExtractRawContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& raw_wall_contours, std::vector<std::vector<cv::Point>>& raw_obstacle_contours)
{
    cv::Mat map = original_map.clone();
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY_INV);
    cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(original_map.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<int> wall_cnt_indices(contours.size());
    std::iota(wall_cnt_indices.begin(), wall_cnt_indices.end(), 0);

//    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs){return contours[lhs].size() > contours[rhs].size();});
    std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs){return cv::contourArea(contours[lhs]) > cv::contourArea(contours[rhs]);});

    std::vector<cv::Point> raw_wall_contour = contours[wall_cnt_indices.front()];
    raw_wall_contours = {raw_wall_contour};

    cv::Mat mask = cv::Mat(original_map.size(), original_map.type(), 255);
    cv::fillPoly(mask, raw_wall_contours, 0);

    cv::Mat base = original_map.clone();
    base += mask;
    cv::threshold(base, base, 128, 255, cv::THRESH_BINARY_INV);

    cv::findContours(base, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    raw_obstacle_contours = contours;
}

/** 默认是空闲区域为白色，障碍物为黑色 **/
void ExtractContours(const cv::Mat& original_map, std::vector<std::vector<cv::Point>>& wall_contours, std::vector<std::vector<cv::Point>>& obstacle_contours, int robot_radius)
{
    ExtractRawContours(original_map, wall_contours, obstacle_contours);

    cv::Mat canvas = cv::Mat(original_map.size(), CV_8UC3, cv::Scalar(255, 255, 255));
    cv::fillPoly(canvas, wall_contours, cv::Scalar(0, 0, 0));
    for(const auto& point:wall_contours.front())
    {
        cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
    }

    cv::fillPoly(canvas, obstacle_contours, cv::Scalar(255, 255, 255));
    for(const auto& obstacle_contour:obstacle_contours)
    {
        for(const auto& point:obstacle_contour)
        {
            cv::circle(canvas, point, robot_radius, cv::Scalar(255, 255, 255), -1);
        }
    }

    cv::Mat canvas_;
    cv::cvtColor(canvas, canvas_, cv::COLOR_BGR2GRAY);
    cv::threshold(canvas_, canvas_, 200, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(robot_radius,robot_radius), cv::Point(-1,-1));
    cv::morphologyEx(canvas_, canvas_, cv::MORPH_OPEN, kernel);

    ExtractRawContours(canvas_, wall_contours, obstacle_contours);



    std::vector<cv::Point> processed_wall_contour;
    cv::approxPolyDP(cv::Mat(wall_contours.front()), processed_wall_contour, 1, true);

    std::vector<std::vector<cv::Point>> processed_obstacle_contours(obstacle_contours.size());
    for(int i = 0; i < obstacle_contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(obstacle_contours[i]), processed_obstacle_contours[i], 1, true);
    }

    wall_contours = {processed_wall_contour};
    obstacle_contours = processed_obstacle_contours;
}

PolygonList ConstructObstacles(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& obstacle_contours)
{
    PolygonList obstacles;
    Polygon obstacle;

    for(const auto& obstacle_contour : obstacle_contours)
    {
        for(int j = 0; j < obstacle_contour.size()-1; j++)
        {
            cv::LineIterator line(original_map, obstacle_contour[j], obstacle_contour[j+1]);
            for(int k = 0; k < line.count-1; k++)
            {
                obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, obstacle_contour[obstacle_contour.size()-1], obstacle_contour[0]);
        for(int j = 0; j < line.count-1; j++)
        {
            obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        obstacles.emplace_back(obstacle);
        obstacle.clear();
    }

    return obstacles;
}

Polygon ConstructDefaultWall(const cv::Mat& original_map)
{
    std::vector<cv::Point> default_wall_contour = {cv::Point(0, 0), cv::Point(0, original_map.rows-1), cv::Point(original_map.cols-1, original_map.rows-1), cv::Point(original_map.cols-1, 0)};
    std::vector<std::vector<cv::Point>>default_wall_contours = {default_wall_contour};

    Polygon default_wall = ConstructObstacles(original_map, default_wall_contours).front();

    return default_wall;
}

Polygon ConstructWall(const cv::Mat& original_map, const std::vector<cv::Point>& wall_contour)
{
    Polygon contour;

    if(!wall_contour.empty())
    {
        for(int i = 0; i < wall_contour.size()-1; i++)
        {
            cv::LineIterator line(original_map, wall_contour[i], wall_contour[i+1]);
            for(int j = 0; j < line.count-1; j++)
            {
                contour.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(original_map, wall_contour.back(), wall_contour.front());
        for(int i = 0; i < line.count-1; i++)
        {
            contour.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        return contour;
    }
    else
    {
        contour = ConstructDefaultWall(original_map);
        return contour;
    }

}

std::vector<CellNode> ConsturctCellGraph(const cv::Mat& original_map, const std::vector<std::vector<cv::Point>>& wall_contours, const std::vector<std::vector<cv::Point>>& obstacle_contours, const Polygon& wall, const PolygonList& obstacles)
{
    cv::Mat3b map = cv::Mat3b(original_map.size());
    map.setTo(cv::Scalar(255, 255, 255));

    cv::fillPoly(map, wall_contours, cv::Scalar(0, 0, 0));
    cv::fillPoly(map, obstacle_contours, cv::Scalar(255, 255, 255));

    std::vector<Event> wall_event_list = GenerateWallEventList(map, wall);
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map, obstacles);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);

    return cell_graph;
}

std::deque<std::deque<Point2D>> StaticPathPlanning(cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& start_point, int robot_radius, bool visualize_cells, bool visualize_path, int color_repeats=10)
{
    std::deque<std::deque<Point2D>> global_path;
    std::deque<Point2D> local_path;
    int corner_indicator = TOPLEFT;

    int start_cell_index = DetermineCellIndex(cell_graph, start_point).front();

    std::deque<Point2D> init_path = WalkInsideCell(cell_graph[start_cell_index], start_point, ComputeCellCornerPoints(cell_graph[start_cell_index])[TOPLEFT]);
    local_path.assign(init_path.begin(), init_path.end());

    std::deque<CellNode> cell_path = GetVisittingPath(cell_graph, start_cell_index);

    if(visualize_cells||visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::imshow("map", map);
    }

    if(visualize_cells)
    {
        std::cout<<"cell graph has "<<cell_graph.size()<<" cells."<<std::endl;
        for(int i = 0; i < cell_graph.size(); i++)
        {
            for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
            {
                std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
            }
        }

        for(const auto& cell : cell_graph)
        {
            DrawCells(map, cell);
            cv::imshow("map", map);
            cv::waitKey(500);
        }
    }

    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);

    if(visualize_path)
    {
        cv::circle(map, cv::Point(start_point.x, start_point.y), 1, cv::Scalar(0, 0, 255), -1);
        for(const auto& point : init_path)
        {
            map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
            UpdateColorMap(JetColorMap);
            cv::imshow("map", map);
            cv::waitKey(1);
        }
    }

    std::deque<Point2D> inner_path;
    std::deque<std::deque<Point2D>> link_path;
    Point2D curr_exit;
    Point2D next_entrance;

    std::deque<int> return_cell_path;
    std::deque<Point2D> return_path;

    for(int i = 0; i < cell_path.size(); i++)
    {
        inner_path = GetBoustrophedonPath(cell_graph, cell_path[i], corner_indicator, robot_radius);
        local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
        if(visualize_path)
        {
            for(const auto& point : inner_path)
            {
                map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                UpdateColorMap(JetColorMap);
                cv::imshow("map", map);
                cv::waitKey(1);
            }
        }

        cell_graph[cell_path[i].cellIndex].isCleaned = true;

        if(i < (cell_path.size()-1))
        {
            curr_exit = inner_path.back();
            next_entrance = FindNextEntrance(curr_exit, cell_path[i+1], corner_indicator);
            link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, cell_path[i], cell_path[i+1]);

            // for debugging
//            std::cout<<std::endl;
//            for(int i = 0; i < link_path.front().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.front()[i]).front();
//                std::cout<<"point lies in curr cell "<<idx<<std::endl;
//            }
//
//            for(int i = 0; i < link_path.back().size(); i++)
//            {
//                int idx = DetermineCellIndex(cell_graph, link_path.back()[i]).front();
//                std::cout<<"point lies in next cell "<<idx<<std::endl;
//            }
//            std::cout<<std::endl;


            local_path.insert(local_path.end(), link_path.front().begin(), link_path.front().end());
            global_path.emplace_back(local_path);
            local_path.clear();
            local_path.insert(local_path.end(), link_path.back().begin(), link_path.back().end());


            if(visualize_path)
            {
                for(const auto& point : link_path.front())
                {
//                    map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", map);
                    cv::waitKey(1);
                }

                for(const auto& point: link_path.back())
                {
//                    map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(255, 255, 255);
                    map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", map);
                    cv::waitKey(1);
                }

            }
        }
    }
    global_path.emplace_back(local_path);

    if(visualize_cells||visualize_path)
    {
        cv::waitKey(0);
    }

    return global_path;
}

std::deque<Point2D> ReturningPathPlanning(cv::Mat& map, std::vector<CellNode>& cell_graph, const Point2D& curr_pos, const Point2D& original_pos, int robot_radius, bool visualize_path)
{
    std::deque<int> return_cell_path = FindShortestPath(cell_graph, curr_pos, original_pos);
    std::deque<Point2D> returning_path;

    if(return_cell_path.size() == 1)
    {
        returning_path = WalkInsideCell(cell_graph[return_cell_path.front()], curr_pos, original_pos);
    }
    else
    {
        returning_path = WalkCrossCells(cell_graph, return_cell_path, curr_pos, original_pos, robot_radius);
    }

    if(visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        for(const auto& point : returning_path)
        {
            map.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(250, 250, 250);
            cv::imshow("map", map);
            cv::waitKey(1);
        }
        cv::waitKey(0);
    }

    return returning_path;
}

std::deque<Point2D> FilterTrajectory(const std::deque<std::deque<Point2D>>& raw_trajectory)
{
    std::deque<Point2D> trajectory;

    for(const auto& sub_trajectory : raw_trajectory)
    {
        for(const auto& position : sub_trajectory)
        {
            if(!trajectory.empty())
            {
                if(position != trajectory.back())
                {
                    trajectory.emplace_back(position);
                }
            }
            else
            {
                trajectory.emplace_back(position);
            }
        }
    }

    return trajectory;
}

void VisualizeTrajectory(cv::Mat& original_map, const std::deque<Point2D>& path, int robot_radius, int vis_mode, int colors=palette_colors)
{
    cv::namedWindow("map", cv::WINDOW_NORMAL);

    std::deque<cv::Scalar> JetColorMap;
    int color_repeated_times = path.size()/colors + 1;
    InitializeColorMap(JetColorMap, color_repeated_times);

    switch (vis_mode)
    {
        case PATH_MODE:
            original_map.at<cv::Vec3b>(path.front().y, path.front().x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
            cv::imshow("map", original_map);
            cv::waitKey(0);

            for(const auto& position:path)
            {
                original_map.at<cv::Vec3b>(position.y, position.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                UpdateColorMap(JetColorMap);
                cv::imshow("map", original_map);
                cv::waitKey(10);
            }
            break;
        case ROBOT_MODE:
            cv::circle(original_map, cv::Point(path.front().x, path.front().y), robot_radius, cv::Scalar(255, 204, 153), -1);
            cv::imshow("map", original_map);
            cv::waitKey(0);

            for(const auto& position:path)
            {
                cv::circle(original_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 204, 153), -1);
                cv::imshow("map", original_map);
                cv::waitKey(10);

                cv::circle(original_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 229, 204), -1);
            }
            break;
        default:
            break;
    }

    cv::waitKey(0);
}






/** 两个输入参数都需要是单位向量, 输出的偏航角为正，则是顺时针旋转，为负则是逆时针旋转 **/
double ComputeYaw(Eigen::Vector2d curr_direction, Eigen::Vector2d base_direction)
{
    double yaw = std::atan2(curr_direction[1], curr_direction[0]) - std::atan2(base_direction[1], base_direction[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    yaw = yaw / M_PI * 180.0;

    return yaw;
}

/** 单位为米 **/
double ComputeDistance(const Point2D& start, const Point2D& end, double meters_per_pix)
{
    double dist = std::sqrt(std::pow((end.x-start.x),2)+std::pow((end.y-start.y),2));
    dist = dist * meters_per_pix;
    return dist;
}

class NavigationMessage
{
public:
    NavigationMessage()
    {
        foward_distance = 0.0;
        global_yaw_angle = 0.0;
        local_yaw_angle = 0.0;
    }
    void SetDistance(double dist)
    {
        foward_distance = dist;
    }
    void SetGlobalYaw(double global_yaw)
    {
        global_yaw_angle = global_yaw;
    }
    void SetLocalYaw(double local_yaw)
    {
        local_yaw_angle = local_yaw;
    }

    double GetDistance()
    {
        return foward_distance;
    }

    double GetGlobalYaw()
    {
        return global_yaw_angle;
    }

    double GetLocalYaw()
    {
        return local_yaw_angle;
    }

    void GetMotion(double& dist, double& global_yaw, double& local_yaw)
    {
        dist = foward_distance;
        global_yaw = global_yaw_angle;
        local_yaw = local_yaw_angle;
    }
    void Reset()
    {
        foward_distance = 0.0;
        global_yaw_angle = 0.0;
        local_yaw_angle = 0.0;
    }

private:
    double foward_distance;
    // 欧拉角表示，逆时针为正，顺时针为负
    double global_yaw_angle;
    double local_yaw_angle;
};

std::vector<NavigationMessage> GetNavigationMessage(const Eigen::Vector2d& curr_direction, std::deque<Point2D> pos_path, double meters_per_pix)
{
    // initialization
    Eigen::Vector2d global_base_direction = {0, -1}; // {x, y}
    Eigen::Vector2d local_base_direction = curr_direction;

    Eigen::Vector2d curr_local_direction;
    Eigen::Vector2d curr_global_direction;

    NavigationMessage message;
    std::vector<NavigationMessage> message_queue;

    double distance = 0.0;
    double step_distance = 0.0;

    double prev_global_yaw = ComputeYaw(curr_direction, global_base_direction);

    double curr_global_yaw = 0.0;
    double curr_local_yaw = 0.0;

    message.SetGlobalYaw(DBL_MAX);
    message.SetLocalYaw(DBL_MAX);

    for(int i = 0; i < pos_path.size()-1; i++)
    {
        if(pos_path[i+1]==pos_path[i])
        {
            continue;
        }
        else
        {
            curr_local_direction = {pos_path[i+1].x-pos_path[i].x, pos_path[i+1].y-pos_path[i].y};
            curr_local_direction.normalize();

            curr_global_yaw = ComputeYaw(curr_local_direction, global_base_direction);
            curr_local_yaw = ComputeYaw(curr_local_direction, local_base_direction);

            if(message.GetGlobalYaw()==DBL_MAX) // initialization
            {
                message.SetGlobalYaw(curr_global_yaw);
            }

            if(message.GetLocalYaw()==DBL_MAX) // initialization
            {
                message.SetLocalYaw(curr_local_yaw);
            }

            if(curr_global_yaw == prev_global_yaw)
            {
                step_distance = ComputeDistance(pos_path[i+1], pos_path[i], meters_per_pix);
                distance += step_distance;
            }
            else
            {
                message.SetDistance(distance);
                message_queue.emplace_back(message);

                message.Reset();
                message.SetGlobalYaw(curr_global_yaw);
                message.SetLocalYaw(curr_local_yaw);

                distance = 0.0;
                step_distance = ComputeDistance(pos_path[i+1], pos_path[i], meters_per_pix);
                distance += step_distance;
            }
            prev_global_yaw = curr_global_yaw;

            local_base_direction = curr_local_direction;
        }
    }

    message.SetDistance(distance);
    message_queue.emplace_back(message);

    return message_queue;
}





const int UP = 0, UPRIGHT = 1, RIGHT = 2, DOWNRIGHT = 3, DOWN = 4, DOWNLEFT = 5, LEFT = 6, UPLEFT = 7, CENTER = 8;
const std::vector<int> map_directions = {UP, UPRIGHT, RIGHT, DOWNRIGHT, DOWN, DOWNLEFT, LEFT, UPLEFT};

int GetFrontDirection(const Point2D& curr_pos, const Point2D& next_pos)
{
    int delta_x = next_pos.x - curr_pos.x;
    int delta_y = next_pos.y - curr_pos.y;

    if(delta_y < 0)
    {
        if(delta_x == 0)
        {
            return UP;
        }
        if(delta_x < 0)
        {
            return UPLEFT;
        }
        if(delta_x > 0)
        {
            return UPRIGHT;
        }
    }

    if(delta_y > 0)
    {
        if(delta_x == 0)
        {
            return DOWN;
        }
        if(delta_x < 0)
        {
            return DOWNLEFT;
        }
        if(delta_x > 0)
        {
            return DOWNRIGHT;
        }
    }

    if(delta_y == 0)
    {
        if(delta_x == 0)
        {
            return CENTER;
        }
        if(delta_x < 0)
        {
            return LEFT;
        }
        if(delta_x > 0)
        {
            return RIGHT;
        }
    }
}

int GetBackDirection(int front_direction)
{
    if(front_direction + 4 >= map_directions.size())
    {
        int index_offset = front_direction + 4 - map_directions.size();
        return map_directions[index_offset];
    }
    else
    {
        return map_directions[front_direction+4];
    }
}

int GetLeftDirection(int front_direction)
{
    if(front_direction - 2 < 0)
    {
        int index_offset = 2 - front_direction;
        return map_directions[map_directions.size()-index_offset];
    }
    else
    {
        return map_directions[front_direction-2];
    }
}

int GetRightDirection(int front_direction)
{
    if(front_direction + 2 >= map_directions.size())
    {
        int index_offset = front_direction + 2 - map_directions.size();
        return map_directions[index_offset];
    }
    else
    {
        return map_directions[front_direction+2];
    }
}

// 模拟机器人尝试旋转, 旋转方向都是顺时针排列
std::vector<int> GetFrontDirectionCandidates(int front_direction)
{
    std::vector<int> front_directions;

    if(front_direction - 2 < 0)
    {
        int index_offset = 2 - front_direction;
        front_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction-2]);
    }

    if(front_direction - 1 < 0)
    {
        int index_offset = 1 - front_direction;
        front_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction-1]);
    }

    front_directions.emplace_back(map_directions[front_direction]);


    if(front_direction + 1 >= map_directions.size())
    {
        int index_offset = front_direction + 1 - map_directions.size();
        front_directions.emplace_back(map_directions[index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction+1]);
    }

    if(front_direction + 2 >= map_directions.size())
    {
        int index_offset = front_direction + 2 - map_directions.size();
        front_directions.emplace_back(map_directions[index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction+2]);
    }

    return front_directions;
}

std::vector<int> GetBackDirectionCandidates(int front_direction)
{
    std::vector<int> back_directions;
    int first_direction = GetRightDirection(front_direction);

    for(int i = 0; i <= 4; i++)
    {
        if(first_direction + i >= map_directions.size())
        {
            int index_offset = first_direction + i - map_directions.size();
            back_directions.emplace_back(map_directions[index_offset]);
        }
        else
        {
            back_directions.emplace_back(map_directions[first_direction + i]);
        }
    }

    return back_directions;
}

std::vector<int> GetLeftDirectionCandidates(int front_direction)
{
    std::vector<int> left_directions;

    for(int i = 4; i >= 0; i--)
    {
        if(front_direction - i < 0)
        {
            int index_offset = i - front_direction;
            left_directions.emplace_back(map_directions[map_directions.size()-index_offset]);
        }
        else
        {
            left_directions.emplace_back(map_directions[front_direction-i]);
        }
    }

    return left_directions;
}

std::vector<int> GetRightDirectionCandidates(int front_direction)
{
    std::vector<int> right_directions;

    for(int i = 0; i <= 4; i++)
    {
        if(front_direction + i >= map_directions.size())
        {
            int index_offset = front_direction + i - map_directions.size();
            right_directions.emplace_back(map_directions[index_offset]);
        }
        else
        {
            right_directions.emplace_back(map_directions[front_direction + i]);
        }
    }

    return right_directions;
}

Point2D GetNextPosition(const Point2D& curr_pos, int direction, int steps)
{
    Point2D next_position;

    switch (direction)
    {
        case UP:
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y - steps);
            return next_position;
        case UPRIGHT:
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y - steps);
            return next_position;
        case RIGHT:
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y);
            return next_position;
        case DOWNRIGHT:
            next_position.x = int(curr_pos.x + steps);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case DOWN:
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case DOWNLEFT:
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y + steps);
            return next_position;
        case LEFT:
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y);
            return next_position;
        case UPLEFT:
            next_position.x = int(curr_pos.x - steps);
            next_position.y = int(curr_pos.y - steps);
            return next_position;
        case CENTER:
            next_position.x = int(curr_pos.x);
            next_position.y = int(curr_pos.y);
            return next_position;
        default:
            return next_position;
    }
}

// 模拟碰撞传感器信号
bool CollisionOccurs(const cv::Mat& map, const Point2D& curr_pos, int detect_direction, int robot_radius)
{
    int obstacle_dist = INT_MAX;
    Point2D ray_pos;

    for(int i = 1; i <= (robot_radius+1); i++)
    {
        ray_pos = GetNextPosition(curr_pos, detect_direction, i);

        if(ray_pos.x < 0 || ray_pos.y < 0 || ray_pos.x >= map.cols || ray_pos.y >= map.rows)
        {
            break;
        }

        if(map.at<cv::Vec3b>(ray_pos.y, ray_pos.x) == cv::Vec3b(255,255,255))
        {
            obstacle_dist = i;
            break;
        }
    }

    return (obstacle_dist == (robot_radius+1));
}

// 结束返回false, 继续则返回true
bool WalkAlongObstacle(const cv::Mat& map,      const Point2D& obstacle_origin,               const Point2D& contouring_origin,
                       int detecting_direction, const std::vector<int>& direction_candidates,
                       Point2D& curr_pos,       int first_turning_direction,                  int second_turning_direction,
                       Polygon& obstacle,       Polygon& new_obstacle,                        bool& isObstacleCompleted,
                       std::deque<Point2D>& contouring_path,                                  int robot_radius)
{
    bool turning = false;
    Point2D last_curr_pos = curr_pos;
    Point2D next_pos;
    Point2D obstacle_point;

    while(!turning)
    {
        for (auto direction: direction_candidates)
        {
            next_pos = GetNextPosition(curr_pos, direction, 1);
            if(next_pos.x < 0 || next_pos.y < 0 || next_pos.x >= map.cols || next_pos.y >= map.rows)
            {
                continue;
            }

            if (CollisionOccurs(map, next_pos, detecting_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, detecting_direction, robot_radius+1);
                if(obstacle_point.x==obstacle_origin.x && obstacle_point.y == obstacle_origin.y)
                {
                    if(!isObstacleCompleted)
                    {
                        obstacle.assign(new_obstacle.begin(), new_obstacle.end());
                        isObstacleCompleted = true;
                    }
                }
                new_obstacle.emplace_back(obstacle_point);
                break;
            }
        }
        if(curr_pos.x==last_curr_pos.x&&curr_pos.y==last_curr_pos.y)
        {
            turning = true;
        }
        else
        {
            last_curr_pos = curr_pos;
        }

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, first_turning_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, second_turning_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;

        if(std::find(contouring_path.begin(), (contouring_path.end()-1), next_pos)!=(contouring_path.end()-1)
        &&contouring_path.size()>1
        &&isObstacleCompleted)
        {
            return false;
        }
    }
    return true;
}

Polygon GetNewObstacle(const cv::Mat& map, Point2D origin, int front_direction, std::deque<Point2D>& contouring_path, int robot_radius)
{
    contouring_path.emplace_back(origin);

    Point2D curr_pos = origin;

    Point2D obstacle_point;
    Polygon new_obstacle;
    Polygon obstacle;

    bool isObstacleCompleted = false;

    int left_direction = GetLeftDirection(front_direction);
    int right_direction = GetRightDirection(front_direction);
    int back_direction = GetBackDirection(front_direction);

    std::deque<int> direcition_list = {right_direction, front_direction, left_direction, back_direction};

    std::vector<int> right_direction_candidates = GetRightDirectionCandidates(front_direction);
    std::vector<int> front_direction_candidates = GetFrontDirectionCandidates(front_direction);
    std::vector<int> left_direction_candidates = GetLeftDirectionCandidates(front_direction);
    std::vector<int> back_direction_candidates = GetBackDirectionCandidates(front_direction);

    std::deque<std::vector<int>> direction_candidates_list = {right_direction_candidates, front_direction_candidates, left_direction_candidates, back_direction_candidates};

    obstacle_point = GetNextPosition(origin, front_direction, robot_radius+1);
    new_obstacle.emplace_back(obstacle_point);

    Point2D obstacle_origin = new_obstacle.front();

    int detecting_direction;
    int first_turning_direction;
    int second_turning_direction;
    int temp_direction;

    bool keepContouring = true;
    std::vector<int> direction_candidates;
    std::vector<int> temp_direction_candidates;


    while(keepContouring)
    {
        direction_candidates = direction_candidates_list[0];
        first_turning_direction = direcition_list[0];
        second_turning_direction = direcition_list[1];
        detecting_direction = direcition_list[1];

        keepContouring = WalkAlongObstacle(map, obstacle_origin, origin, detecting_direction, direction_candidates, curr_pos, first_turning_direction, second_turning_direction
                , obstacle, new_obstacle, isObstacleCompleted, contouring_path, robot_radius);

        temp_direction = direcition_list.front();
        direcition_list.pop_front();
        direcition_list.emplace_back(temp_direction);

        temp_direction_candidates = direction_candidates_list.front();
        direction_candidates_list.pop_front();
        direction_candidates_list.emplace_back(temp_direction_candidates);
    }

    contouring_path.pop_front();

    return obstacle;

} //考虑使用contouring path当障碍物

int GetCleaningDirection(const CellNode& cell, Point2D exit)
{
    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell);

//    if(exit.x == corner_points[TOPLEFT].x && exit.y == corner_points[TOPLEFT].y)
//    {
//        return LEFT;
//    }
//    if(exit.x == corner_points[BOTTOMLEFT].x && exit.y == corner_points[BOTTOMLEFT].y)
//    {
//        return LEFT;
//    }
//    if(exit.x == corner_points[TOPRIGHT].x && exit.y == corner_points[TOPRIGHT].y)
//    {
//        return RIGHT;
//    }
//    if(exit.x == corner_points[BOTTOMRIGHT].x && exit.y == corner_points[BOTTOMRIGHT].y)
//    {
//        return RIGHT;
//    }

    double dist_to_left = std::abs(exit.x - corner_points[TOPLEFT].x);
    double dist_to_right = std::abs(exit.x = corner_points[TOPRIGHT].x);

    if(dist_to_left >= dist_to_right)
    {
        return RIGHT;
    }
    else
    {
        return LEFT;
    }

}

// 清扫方向只分向左和向右
std::deque<std::deque<Point2D>> LocalReplanning(cv::Mat& map, CellNode outer_cell, const PolygonList& obstacles, const Point2D& curr_pos, std::vector<CellNode>& curr_cell_graph, int cleaning_direction, int robot_radius, bool visualize_cells=false, bool visualize_path=false)
{
    //TODO: 边界判断
    int start_x = INT_MAX;
    int end_x = INT_MAX;

    if(cleaning_direction == LEFT)
    {
        start_x = outer_cell.ceiling.front().x;

        if(curr_pos.x + 2*(robot_radius + 1) <= outer_cell.ceiling.back().x)
        {
            end_x =  curr_pos.x + 2*(robot_radius + 1);
        }
        else
        {
            end_x = outer_cell.ceiling.back().x;
        }
    }
    if(cleaning_direction == RIGHT)
    {
        end_x = outer_cell.ceiling.back().x;

        if(curr_pos.x - 2*(robot_radius + 1) >= outer_cell.ceiling.front().x)
        {
            start_x = curr_pos.x - 2*(robot_radius + 1);
        }
        else
        {
            start_x = outer_cell.ceiling.front().x;
        }
    }
    int outer_cell_start_index_offset = start_x - outer_cell.ceiling.front().x;
    int outer_cell_end_index_offset = end_x - outer_cell.ceiling.front().x;

    CellNode inner_cell;
    for(int i = outer_cell_start_index_offset; i <= outer_cell_end_index_offset; i++)
    {
        inner_cell.ceiling.emplace_back(outer_cell.ceiling[i]);
        inner_cell.floor.emplace_back(outer_cell.floor[i]);
    }

//    curr_cell_graph = GenerateCells(map, inner_cell, obstacles);   // 这里需要改
    std::deque<std::deque<Point2D>> replanning_path = StaticPathPlanning(map, curr_cell_graph, curr_pos, robot_radius, visualize_cells, visualize_path);

    return replanning_path;
} //回退区域需要几个r+1

// 每一段都是在一个cell中的路径
std::deque<Point2D> DynamicPathPlanning(cv::Mat& map, const std::vector<CellNode>& global_cell_graph, std::deque<std::deque<Point2D>> global_path, int robot_radius, bool returning_home, bool visualize_path, int color_repeats=10)
{
    std::deque<Point2D> dynamic_path;

    std::deque<std::deque<Point2D>> curr_path;
    std::deque<Point2D> curr_sub_path;
    std::deque<Point2D> contouring_path;

    std::deque<std::deque<Point2D>> replanning_path;
    std::deque<std::deque<Point2D>> remaining_curr_path;

    std::deque<Point2D> linking_path;

    Point2D curr_pos;
    Point2D next_pos;
    Point2D curr_exit;

    int front_direction;
    int cleaning_direction;

    Polygon new_obstacle;
//    Polygon temp_new_obstacle;

    int curr_cell_index;
    CellNode curr_cell;

    std::vector<CellNode> curr_cell_graph;

    PolygonList overall_obstacles;
    PolygonList curr_obstacles;
    std::vector<cv::Point> visited_obstacle_contour;
    std::vector<std::vector<cv::Point>> visited_obstacle_contours;

    std::vector<std::deque<std::deque<Point2D>>> unvisited_paths = {global_path};
    std::vector<std::vector<CellNode>> cell_graph_list = {global_cell_graph};
    std::vector<Point2D> exit_list = {global_path.back().back()};

    cv::Mat vismap = map.clone();
    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);
    if(visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::imshow("map", vismap);
    }


    while(!unvisited_paths.empty() && !cell_graph_list.empty())
    {
        curr_path = unvisited_paths.back();
        curr_cell_graph = cell_graph_list.back();

        for(int i = 0; i < curr_path.size(); i++)
        {
            curr_sub_path.assign(curr_path[i].begin(), curr_path[i].end());

            for(int j = 0; j < curr_sub_path.size()-1; j++)
            {
                curr_pos = curr_sub_path[j];
                next_pos = curr_sub_path[j+1];
                dynamic_path.emplace_back(curr_pos);

                if(visualize_path)
                {
                    vismap.at<cv::Vec3b>(curr_pos.y, curr_pos.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vismap);
                    cv::waitKey(1);
                }

                front_direction = GetFrontDirection(curr_pos, next_pos);
                if(CollisionOccurs(map, curr_pos, front_direction, robot_radius))
                {
                    new_obstacle = GetNewObstacle(map, curr_pos, front_direction, contouring_path, robot_radius);
//                    new_obstacle = GetSingleContouringArea(map, temp_new_obstacle, robot_radius);
                    overall_obstacles.emplace_back(new_obstacle);

                    // for debugging
//                    for(int i = 0; i < new_obstacle.size(); i++)
//                    {
//                        std::cout<<"x:"<< new_obstacle[i].x <<", y:"<< new_obstacle[i].y <<std::endl;
//                    }
//
//                    for(int i = 0; i < new_obstacle.size(); i++)
//                    {
//                        vismap.at<cv::Vec3b>(new_obstacle[i].y, new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
//                    }
//                    for(int i = 0; i < contouring_path.size(); i++)
//                    {
//                        vismap.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(255, 0, 0);
//                    }
//                    cv::circle(map, cv::Point(contouring_path.front().x, contouring_path.front().y), 2, cv::Scalar(0, 255, 255), -1);
//                    cv::circle(map, cv::Point(contouring_path.back().x, contouring_path.back().y), 2, cv::Scalar(255, 0, 255), -1);
//                    PointTypeTest(map, new_obstacle);
//                    cv::imshow("map", vismap);
//                    cv::waitKey(0);

                    dynamic_path.insert(dynamic_path.end(), contouring_path.begin(), contouring_path.end());

                    if(visualize_path)
                    {
                        for(const auto& point : contouring_path)
                        {
                            vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                            UpdateColorMap(JetColorMap);
                            cv::imshow("map", vismap);
                            cv::waitKey(1);
                        }
                    }

                    contouring_path.clear();

                    visited_obstacle_contour.clear();
                    visited_obstacle_contours.clear();

                    for(const auto& point: new_obstacle)
                    {
                        visited_obstacle_contour.emplace_back(cv::Point(point.x,point.y));
                    }

                    visited_obstacle_contours.emplace_back(visited_obstacle_contour);

                    curr_cell_index = DetermineCellIndex(curr_cell_graph, curr_pos).front();
                    curr_cell = curr_cell_graph[curr_cell_index];
                    curr_obstacles={new_obstacle};
                    curr_exit = curr_sub_path.back();

                    // for debugging
//                    DrawCells(map, curr_cell, cv::Scalar(255, 0, 255));
//                    vismap.at<cv::Vec3b>(curr_exit.y, curr_exit.x)=cv::Vec3b(0,255,255);
//                    cv::imshow("map", vismap);
//                    cv::waitKey(0);

                    cleaning_direction = GetCleaningDirection(curr_cell, curr_exit);

                    replanning_path = LocalReplanning(map, curr_cell, curr_obstacles, dynamic_path.back(), curr_cell_graph, cleaning_direction, robot_radius, false, false); // 此处会更新curr_cell_graph
                    cv::fillPoly(map, visited_obstacle_contours, cv::Scalar(50, 50, 50));
                    cv::fillPoly(vismap, visited_obstacle_contours, cv::Scalar(50, 50, 50));

                    remaining_curr_path.assign(curr_path.begin()+i+1, curr_path.end());

                    goto UPDATING_REMAINING_PATHS;
                }
            }
        }

        if(dynamic_path.back().x != exit_list.back().x && dynamic_path.back().y != exit_list.back().y)
        {
            linking_path = ReturningPathPlanning(map, cell_graph_list.back(), dynamic_path.back(), exit_list.back(), robot_radius, false);
            dynamic_path.insert(dynamic_path.end(), linking_path.begin(), linking_path.end());

            if(visualize_path)
            {
                for(const auto& point : linking_path)
                {
                    vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(uchar(JetColorMap.front()[0]),uchar(JetColorMap.front()[1]),uchar(JetColorMap.front()[2]));
                    UpdateColorMap(JetColorMap);
                    cv::imshow("map", vismap);
                    cv::waitKey(1);
                }
            }
        }

        exit_list.pop_back();
        unvisited_paths.pop_back();
        cell_graph_list.pop_back();
        continue;

        UPDATING_REMAINING_PATHS:
        exit_list.emplace_back(curr_exit);
        unvisited_paths.pop_back();
        unvisited_paths.emplace_back(remaining_curr_path);
        unvisited_paths.emplace_back(replanning_path);
        cell_graph_list.emplace_back(curr_cell_graph);
    }

    if(returning_home)
    {
        cv::Mat3b returning_map = cv::Mat3b(map.size(), CV_8U);
        returning_map.setTo(cv::Scalar(0, 0, 0));
        std::vector<cv::Point> returning_obstacle_contour;
        std::vector<std::vector<cv::Point>> returning_obstacle_contours;

        for(const auto& obstacle: overall_obstacles)
        {
            for(const auto& point : obstacle)
            {
                returning_obstacle_contour.emplace_back(cv::Point(point.x,point.y));
            }
            returning_obstacle_contours.emplace_back(returning_obstacle_contour);
            returning_obstacle_contour.clear();
        }

        cv::fillPoly(returning_map, returning_obstacle_contours, cv::Scalar(255, 255, 255));

        Polygon returning_map_border = ConstructDefaultWall(returning_map);
//        std::vector<CellNode> returning_cell_graph = GenerateCells(returning_map, returning_map_border, overall_obstacles);//这里需要改
        std::vector<CellNode> returning_cell_graph; //这里需要改

        // for debugging
//        for(auto cell:returning_cell_graph)
//        {
//            DrawCells(vismap, cell, cv::Scalar(0, 255, 255));
//            cv::imshow("map", vismap);
//            cv::waitKey(0);
//        }
        //

        for(auto cell : returning_cell_graph)
        {
            cell.isCleaned = true;
        }

        std::deque<Point2D> returning_path = ReturningPathPlanning(returning_map, returning_cell_graph, dynamic_path.back(), dynamic_path.front(), robot_radius, false);

        if(visualize_path)
        {
            for(const auto& point : returning_path)
            {
                vismap.at<cv::Vec3b>(point.y, point.x)=cv::Vec3b(250,250,250);
                cv::imshow("map", vismap);
                cv::waitKey(1);
            }
        }

        dynamic_path.insert(dynamic_path.end(), returning_path.begin(), returning_path.end());
    }

    if(visualize_path)
    {
        cv::waitKey(5000);
    }

    return dynamic_path;
}













// 静态地图路径规划测试多边形
std::vector<cv::Point> handcrafted_polygon_1_1 = {cv::Point(200,300), cv::Point(300,200), cv::Point(200,100), cv::Point(100,200)};
std::vector<cv::Point> handcrafted_polygon_1_2 = {cv::Point(300,350), cv::Point(350,300), cv::Point(300,250), cv::Point(250,300)};

std::vector<cv::Point> handcrafted_polygon_2 = {cv::Point(125,125), cv::Point(125,175), cv::Point(225,175), cv::Point(225,225),
                                                cv::Point(175,250), cv::Point(225,300), cv::Point(125,325), cv::Point(125,375),
                                                cv::Point(375,375), cv::Point(375,325), cv::Point(275,325), cv::Point(275,275),
                                                cv::Point(325,250), cv::Point(275,200), cv::Point(375,175), cv::Point(375,125)};

std::vector<cv::Point> handcrafted_polygon_3 = {cv::Point(100,100), cv::Point(100,500), cv::Point(150,500), cv::Point(150,150),
                                                cv::Point(450,150), cv::Point(450,300), cv::Point(300,300), cv::Point(300,250),
                                                cv::Point(350,250), cv::Point(350,200), cv::Point(250,200), cv::Point(250,350),
                                                cv::Point(500,350), cv::Point(500,100)};

std::vector<cv::Point> handcrafted_polygon_4_1 = {cv::Point(20,20),  cv::Point(20,200), cv::Point(100,200),cv::Point(100,399),
                                                  cv::Point(20,399), cv::Point(20, 579),cv::Point(200,579),cv::Point(200,499),cv::Point(399,499),cv::Point(399,579),
                                                  cv::Point(579,579),cv::Point(579,399),cv::Point(499,399),cv::Point(499,200),cv::Point(579,200),cv::Point(579,20),
                                                  cv::Point(349,20), cv::Point(349,100),cv::Point(250,100),cv::Point(250,20)};
std::vector<cv::Point> handcrafted_polygon_4_2 = {cv::Point(220,220),cv::Point(220,380),cv::Point(380,380),cv::Point(380,220)};


// 动态地图路径规划测试多边形
std::vector<cv::Point> handcrafted_polygon_5_1 = {cv::Point(125, 50), cv::Point(50, 125), cv::Point(125, 200), cv::Point(200, 125)};
std::vector<cv::Point> handcrafted_polygon_5_2 = {cv::Point(80, 300), cv::Point(80, 400), cv::Point(160, 400), cv::Point(120, 350),
                                                  cv::Point(160, 300)};
std::vector<cv::Point> handcrafted_polygon_5_3 = {cv::Point(100, 450), cv::Point(100, 550), cv::Point(140, 550), cv::Point(140, 450)};
std::vector<cv::Point> handcrafted_polygon_5_4 = {cv::Point(300, 150), cv::Point(300, 250), cv::Point(400, 220), cv::Point(400, 180)};



void VisualizeObstaclePointType(cv::Mat& map, const Polygon& obstacle)
{
    PolygonList obstacles = {obstacle};

    std::vector<Event> event_list = GenerateObstacleEventList(map, obstacles);

    for(auto event: event_list)
    {
        if(event.event_type == IN)
        {
            std::cout<<event.x<<", "<<event.y<<", IN"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == OUT)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_IN)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_OUT)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_TOP)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_TOP"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_BOTTOM)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == MIDDLE)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50 ,50);
        }
        if(event.event_type == CEILING)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255 ,255);
        }
        if(event.event_type == FLOOR)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0 ,0);
        }
    }
}

void VisualizeWallPointType(cv::Mat& map, const Polygon& wall)
{
    std::vector<Event> event_list = GenerateWallEventList(map, wall);
    for(auto event: event_list)
    {
        if(event.event_type == IN_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == IN_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", IN_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == OUT_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == OUT_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", OUT_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_IN_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_IN_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_IN_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
        }
        if(event.event_type == INNER_OUT_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_TOP_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_TOP_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == INNER_OUT_BOTTOM_EX)
        {
            std::cout<<event.x<<", "<<event.y<<", INNER_OUT_BOTTOM_EX"<<std::endl;
            cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
        }
        if(event.event_type == MIDDLE)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50 ,50);
        }
        if(event.event_type == CEILING)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255 ,255);
        }
        if(event.event_type == FLOOR)
        {
            map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0 ,0);
        }
    }
}





void CheckPath(const std::deque<Point2D>& path)
{
    int breakpoints = 0;
    int duplicates = 0;

    for(int i = 1; i < path.size(); i++)
    {
        if(std::abs(path[WrappedIndex((i-1),path.size())].x-path[i].x)>1||std::abs(path[WrappedIndex((i-1),path.size())].y-path[i].y)>1)
        {
            breakpoints++;
            std::cout<<"break points :"<<path[WrappedIndex((i-1),path.size())].x<<", "<<path[WrappedIndex((i-1),path.size())].y
                     <<"---->"<<path[i].x<<", "<<path[i].y<<std::endl;
        }
        if(path[WrappedIndex((i-1),path.size())]==path[i])
        {
            duplicates++;
        }

    }
    std::cout<<"breakpoints: "<<breakpoints<<std::endl;
    std::cout<<"duplicates: "<<duplicates<<std::endl;
}

//void MoveAsPathPlannedTest(cv::Mat& map, double meters_per_pix, const Point2D& start, const std::vector<NavigationMessage>& motion_commands)
//{
//    int pixs;
//    Point2D begin = start, end;
//
//    cv::namedWindow("original_map", cv::WINDOW_NORMAL);
//
//    for(auto command:motion_commands)
//    {
//        pixs = int(command.GetDistance()/meters_per_pix);
//        if(command.GetGlobalYaw()==0.0)
//        {
//            end = Point2D(begin.x, begin.y-pixs);
//            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 0, 255));
//            cv::imshow("original_map", map);
//            cv::waitKey(100);
//        }
//        if(command.GetGlobalYaw()==180.0)
//        {
//            end = Point2D(begin.x, begin.y+pixs);
//            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(255, 0, 0));
//            cv::imshow("original_map", map);
//            cv::waitKey(100);
//        }
//        if(command.GetGlobalYaw()==90.0)
//        {
//            end = Point2D(begin.x+pixs, begin.y);
//            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 255, 0));
//            cv::imshow("original_map", map);
//            cv::waitKey(100);
//        }
//        if(command.GetGlobalYaw()==-90.0)
//        {
//            end = Point2D(begin.x-pixs, begin.y);
//            cv::line(map, cv::Point(begin.x, begin.y), cv::Point(end.x, end.y), cv::Scalar(0, 255, 255));
//            cv::imshow("original_map", map);
//            cv::waitKey(100);
//        }
//        begin = end;
//    }
//    cv::waitKey(0);
//}






//void GetNewObstacleTest()
//{
//
//
//    Point2D collision_point = Point2D(80, 294); // 80 406; 80, 294; 166, 350; 74, 350
//    std::deque<Point2D> contouring_path;
//    Polygon new_obstacle = GetNewObstacle(curr_map, collision_point, DOWN, contouring_path, robot_radius); // UP ; DOWN, LEFT, RIGHT
//    for(int i = 0; i < contouring_path.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(0, 0, 255);
//    }
//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(new_obstacle[i].y, new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
//    }
//
////    std::cout<<"first in contouring path: "<<contouring_path.front().x<<", "<<contouring_path.front().y<<std::endl;
////    std::cout<<"last in contouring path: "<<contouring_path.back().x<<", "<<contouring_path.back().y<<std::endl;
//    std::cout<<"contouring path point num: "<<contouring_path.size()<<std::endl;
//
////    for(int i = 0; i < contouring_path.size(); i++)
////    {
////        std::cout<<"x: "<<contouring_path[i].x<<", y: "<<contouring_path[i].y<<std::endl;
////    }
//
//
//    std::cout<<"point num: "<<new_obstacle.size()<<std::endl;
//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        std::cout<<"x: "<<new_obstacle[i].x<<", y: "<<new_obstacle[i].y<<std::endl;
//    }
//
//
//
//
////    std::vector<CellNode> curr_cell_graph;
////    PolygonList new_obstacles = {new_obstacle};
////    std::deque<std::deque<Point2D>> replanning_path = LocalReplanning(curr_map, original_cell_graph[2], new_obstacles, collision_point, curr_cell_graph, RIGHT, robot_radius);
////
////    for(int i = 0; i < replanning_path.size(); i++)
////    {
////        for(int j = 0; j < replanning_path[i].size(); j++)
////        {
////            curr_map.at<cv::Vec3b>(replanning_path[i][j].y,replanning_path[i][j].x)=cv::Vec3b(0, 255, 255);
////            cv::imshow("map", curr_map);
////            cv::waitKey(0);
////        }
////    }
//
//
//
//
////    PointTypeTest(curr_map, new_obstacle, robot_radius);
//    cv::namedWindow("map", cv::WINDOW_NORMAL);
//    cv::imshow("map", curr_map);
//    cv::waitKey(0);
//
//
//
//
//
//}

int main()
{

//  test for real pics

//    double meters_per_pix = 0.02;
//
//    double robot_size_in_meters = 0.15;
//
//    int robot_radius = int(robot_size_in_meters / meters_per_pix);
//
//    cv::namedWindow("map", cv::WINDOW_NORMAL);
//
//    cv::Mat image = cv::imread("../map.png", CV_8UC1);
//
//    cv::Mat vis_map;
//    cv::cvtColor(image, vis_map, cv::COLOR_GRAY2BGR);
//    vis_map.convertTo(vis_map, CV_8UC3);
//
//    std::vector<std::vector<cv::Point>> obstacle_contours;
//    std::vector<std::vector<cv::Point>> wall_contours;
//
//    ExtractContours(image, wall_contours, obstacle_contours, robot_radius);
//
//    cv::Mat map = cv::Mat(image.size(), CV_8UC3, cv::Scalar(255, 255, 255));
//
//    cv::fillPoly(map, wall_contours, cv::Scalar(0, 0, 0));
//
//    Polygon external_contour = ConstructWall(map, wall_contours.front());
//
//    std::vector<Event> wall_event_list = GenerateWallEventList(map, external_contour);
//    std::vector<Event> obstacle_event_list;
//    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);
//
//    std::vector<CellNode> cell_graph;
//    std::vector<int> cell_index_slice;
//    std::vector<int> original_cell_index_slice;
//    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);
//
//
//
//    Point2D start = Point2D(map.cols/2, map.rows/2);
//    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false);
//
////    for debugging
////    for(int i = 0; i < original_planning_path.size(); i++)
////    {
////        for(int j = 0; j < original_planning_path[i].size(); j++)
////        {
////            std::cout<<original_planning_path[i][j].x<<", "<<original_planning_path[i][j].y<<std::endl;
////        }
////        std::cout<<std::endl;
//
//
//    std::deque<Point2D> path = FilterTrajectory(original_planning_path);
//
//    std::cout<<"path length: "<<path.size()<<std::endl;
//
//    CheckPath(path);
//
//    VisualizeTrajectory(vis_map, path, robot_radius, PATH_MODE);
//
//
//    Eigen::Vector2d curr_direction = {0, -1};
//    std::vector<NavigationMessage> messages = GetNavigationMessage(curr_direction, path, meters_per_pix);
//
//    double dist, global_yaw, local_yaw;
//    for(int i = 0; i < messages.size(); i++)
//    {
//        messages[i].GetMotion(dist, global_yaw, local_yaw);
//        std::cout<<"globally rotate "<<global_yaw<<" degree(locally rotate "<<local_yaw<<" degree) and go forward for "<<dist<<" m."<<std::endl;
//    }






//  test for extract wall contour and obstacle contour

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::Mat1b original_image = cv::imread("../complicate_map.png", CV_8U); // 默认是空闲区域为白色，障碍物为黑色

    original_image = PreprocessMap(original_image);

////  for debugging
////    cv::Mat canvas = cv::Mat::zeros(original_image.size(), CV_8UC3);
////    for(int i = 0; i<=contours.size()-1; i++)
////    {
////        cv::drawContours(canvas, contours, i, cv::Scalar(255, 0, 0));
////        cv::imshow("map", canvas);
////        cv::waitKey(0);
////    }

    // test results accquired above

    int robot_radius = 5;

    std::vector<std::vector<cv::Point>> wall_contours;
    std::vector<std::vector<cv::Point>> obstacle_contours;
    ExtractRawContours(original_image, wall_contours, obstacle_contours);

    cv::Mat3b map = cv::Mat3b(original_image.size());
    map.setTo(cv::Scalar(255, 255, 255));

    cv::fillPoly(map, wall_contours, cv::Scalar(0, 0, 0));
    cv::fillPoly(map, obstacle_contours, cv::Scalar(255, 255, 255));

    Polygon wall = ConstructWall(map, wall_contours.front());
    PolygonList obstacles = ConstructObstacles(map, obstacle_contours);

//  for debugging
//    for(auto inner_contour:inner_contours)
//    {
//        PointTypeTest(map, inner_contour);
//        cv::imshow("map", map);
//        cv::waitKey(0);
//        std::cout<<std::endl;
//    }
//
//    PointTypeTestExternal(map, external_contour);
//    cv::imshow("map", map);
//    cv::waitKey(0);


    std::vector<Event> wall_event_list = GenerateWallEventList(map, wall);
    std::vector<Event> obstacle_event_list = GenerateObstacleEventList(map, obstacles);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(wall_event_list, obstacle_event_list);

//    for debugging
//    std::deque<Event> slice;
//    for(int i = 0; i < slice_list.size(); i++)
//    {
//        slice = FilterSlice(slice_list[i]);
//        std::cout<<"slice "<<i<<": ";
//        for(int j = 0; j < slice.size(); j++)
//        {
//            EventType type = slice[j].event_type;
//            switch (type)
//            {
//                case IN:
//                    std::cout<<"IN; ";
//                    break;
//                case IN_TOP:
//                    std::cout<<"IN_TOP; ";
//                    break;
//                case IN_BOTTOM:
//                    std::cout<<"IN_BOTTOM; ";
//                    break;
//                case OUT:
//                    std::cout<<"OUT; ";
//                    break;
//                case OUT_TOP:
//                    std::cout<<"OUT_TOP; ";
//                    break;
//                case OUT_BOTTOM:
//                    std::cout<<"OUT_BOTTOM; ";
//                    break;
//                case INNER_IN:
//                    std::cout<<"INNER_IN; ";
//                    break;
//                case INNER_IN_TOP:
//                    std::cout<<"INNER_IN_TOP; ";
//                    break;
//                case INNER_IN_BOTTOM:
//                    std::cout<<"INNER_IN_BOTTOM; ";
//                    break;
//                case INNER_OUT:
//                    std::cout<<"INNER_OUT; ";
//                    break;
//                case INNER_OUT_TOP:
//                    std::cout<<"INNER_OUT_TOP; ";
//                    break;
//                case INNER_OUT_BOTTOM:
//                    std::cout<<"INNER_OUT_BOTTOM; ";
//                    break;
//                case IN_EX:
//                    std::cout<<"IN_EX; ";
//                    break;
//                case IN_TOP_EX:
//                    std::cout<<"IN_TOP_EX; ";
//                    break;
//                case IN_BOTTOM_EX:
//                    std::cout<<"IN_BOTTOM_EX; ";
//                    break;
//                case OUT_EX:
//                    std::cout<<"OUT_EX; ";
//                    break;
//                case OUT_TOP_EX:
//                    std::cout<<"OUT_TOP_EX; ";
//                    break;
//                case OUT_BOTTOM_EX:
//                    std::cout<<"OUT_BOTTOM_EX; ";
//                    break;
//                case INNER_IN_EX:
//                    std::cout<<"INNER_IN_EX; ";
//                    break;
//                case INNER_IN_TOP_EX:
//                    std::cout<<"INNER_IN_TOP_EX; ";
//                    break;
//                case INNER_IN_BOTTOM_EX:
//                    std::cout<<"INNER_IN_BOTTOM_EX; ";
//                    break;
//                case INNER_OUT_EX:
//                    std::cout<<"INNER_OUT_EX; ";
//                    break;
//                case INNER_OUT_TOP_EX:
//                    std::cout<<"INNER_OUT_TOP_EX; ";
//                    break;
//                case INNER_OUT_BOTTOM_EX:
//                    std::cout<<"INNER_OUT_BOTTOM_EX; ";
//                    break;
//                case MIDDLE:
//                    std::cout<<"MIDDLE; ";
//                    break;
//                case CEILING:
//                    std::cout<<"CEILING; ";
//                    break;
//                case FLOOR:
//                    std::cout<<"FLOOR; ";
//                    break;
//                case UNALLOCATED:
//                    std::cout<<"UNALLOCATED; ";
//                    break;
//            }
//        }
//        std::cout<<std::endl;
//    }

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    ExecuteCellDecomposition(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);

//    for(int i = 0; i < cell_graph.size(); i++)
//    {
//        DrawCells(map, cell_graph[i], cv::Scalar(255, 0, 255));
//        cv::imshow("map", map);
//        cv::waitKey(0);
//    }

    cv::imshow("map", map);
    cv::waitKey(0);

    Point2D start = cell_graph.front().ceiling.front();
    int color_repeated_times = 20;
    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, true, color_repeated_times);





//  Static test for wall+obstacle

//    int robot_radius = 20;
//
//    cv::Mat map = cv::Mat(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
//
//    std::vector<cv::Point> wall_contour = {cv::Point(20,20),cv::Point(20,200),cv::Point(100,200),cv::Point(100,399),
//                cv::Point(20,399),cv::Point(20, 579),cv::Point(200,579),cv::Point(200,499),cv::Point(399,499),cv::Point(399,579),
//                cv::Point(579,579),cv::Point(579,399),cv::Point(499,399),cv::Point(499,200),cv::Point(579,200),cv::Point(579,20),
//                cv::Point(349,20),cv::Point(349,100),cv::Point(250,100),cv::Point(250,20)};
//
//    std::vector<std::vector<cv::Point>> wall_contours = {wall_contour};
//    cv::fillPoly(map, wall_contours, cv::Scalar(0, 0, 0));
//
//    Polygon external_contour = ConstructCave(map, wall_contour);
//
//    std::vector<cv::Point> obstacle_contour = {cv::Point(220,220),cv::Point(220,380),cv::Point(380,380),cv::Point(380,220)};
//    std::vector<std::vector<cv::Point>> obstacle_contours = {obstacle_contour};
//    cv::fillPoly(map, obstacle_contours, cv::Scalar(255, 255, 255));
//
//    cv::Mat original_map = map.clone();
//
//    PolygonList inner_contours = ConstructObstacles(map, obstacle_contours);
//
////    PointTypeTestExternal(map, external_contour);
//
//    std::vector<Event> external_event_list = EventListGeneratorExternal(map, external_contour);
//    std::vector<Event> inner_event_list = EventListGenerator(map, inner_contours);
//
//    std::vector<Event> event_list;
//    event_list.insert(event_list.end(), external_event_list.begin(), external_event_list.end());
//    event_list.insert(event_list.end(), inner_event_list.begin(), inner_event_list.end());
//    std::sort(event_list.begin(), event_list.end());
//
//    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
//
//    std::vector<CellNode> cell_graph;
//    std::vector<int> cell_index_slice;
//    std::vector<int> original_cell_index_slice;
//    ExecuteCellDecompositionOverall(cell_graph, cell_index_slice, original_cell_index_slice, slice_list);
//
////    for(int i = 0; i < cell_graph.size(); i++)
////    {
////        DrawCells(map, cell_graph[i], cv::Scalar(0, 255, 255));
////        cv::imshow("map", map);
////        cv::waitKey(0);
////    }
//
//    cv::imshow("map", map);
//    cv::waitKey(0);
//
//    Point2D start = Point2D(200, 200);
//    int color_repeated_times = 50;
//    std::deque<std::deque<Point2D>> original_planning_path = StaticPathPlanning(map, cell_graph, start, robot_radius, false, false, color_repeated_times);
//
//    std::deque<Point2D> path;
//    for(int i = 0; i < original_planning_path.size(); i++)
//    {
//        path.insert(path.end(), original_planning_path[i].begin(), original_planning_path[i].end());
//    }
//    double meters_per_pix = 0.02;
//    Eigen::Vector2d curr_direction = {0, -1};
//    std::vector<NavigationMessage> messages = GetNavigationMessage(curr_direction, path, meters_per_pix);
//
//    double dist, global_yaw, local_yaw;
//    for(int i = 0; i < messages.size(); i++)
//    {
//        messages[i].GetMotion(dist, global_yaw, local_yaw);
//        std::cout<<"globally rotate "<<global_yaw<<" degree(locally rotate "<<local_yaw<<" degree) and go forward for "<<dist<<" m."<<std::endl;
//    }
//
//    MoveAsPathPlannedTest(original_map, meters_per_pix, start, messages);






//    Contouring path test
//    cv::namedWindow("map", cv::WINDOW_NORMAL);

//    Point2D collision_point = Point2D(325, 367); //(400, 314) (325, 367)
//    std::deque<Point2D> contouring_path;
//    Polygon new_obstacle = GetNewObstacle(curr_map, collision_point, DOWN, contouring_path, robot_radius);

//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(new_obstacle[i].y, new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
//    }
//    for(int i = 0; i < contouring_path.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(255, 0, 0);
//    }

//    PointTypeTest(curr_map, new_obstacle);

//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        std::cout<<"x:"<< new_obstacle[i].x <<", y:"<< new_obstacle[i].y <<std::endl;
//    }

//    PolygonList obstacles = {new_obstacle};
//    Polygon map_border = ConstructDefaultMapBorder(curr_map);
//    std::vector<CellNode> cell_graph = GenerateCells(curr_map, map_border, obstacles);
//    for(int i = 0; i < cell_graph.size(); i++)
//    {
//        DrawCells(curr_map, cell_graph[i], cv::Scalar(0, 255, 255));
//        cv::imshow("map", curr_map);
//        cv::waitKey(0);
//    }
//    cv::imshow("map", curr_map);
//    cv::waitKey(0);
//    std::deque<std::deque<Point2D>> path = StaticPathPlanning(curr_map, cell_graph, Point2D(1,1), robot_radius, true, true, 50);

    return 0;
}