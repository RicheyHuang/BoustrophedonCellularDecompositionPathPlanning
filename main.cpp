#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//cv::Mat map;

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
    MIDDLE,
    CEILING,
    FLOOR,
    UNALLOCATED
};

const int TOPLEFT = 0;
const int BOTTOMLEFT = 1;
const int BOTTOMRIGHT = 2;
const int TOPRIGHT = 3;

class Point2D
{
public:
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
    Point2D()
    {
        x = INT_MAX;
        y = INT_MAX;
    }
    int x;
    int y;
};

typedef std::vector<Point2D> Polygon;   // contour points extracted from a blob, sorted by counter clockwise manner
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

class CellNode  // ceiling.size() == floor.size()
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;
    Edge ceiling;
    Edge floor;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};

void WalkingThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path)  // Depth First Search Method
{
    if(!cell_graph[cell_index].isVisited)
    {
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);
    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;

    CellNode neighbor;
    int neighbor_idx;

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
        WalkingThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
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
            WalkingThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}

std::deque<CellNode> GetVisittingPath(std::vector<CellNode>& cell_graph, int first_cell_index)
{
    int unvisited_counter = cell_graph.size();
    std::deque<CellNode> visitting_path;
    WalkingThroughGraph(cell_graph, first_cell_index, unvisited_counter, visitting_path);
    std::reverse(visitting_path.begin(), visitting_path.end());
    return visitting_path;
}

std::vector<Point2D> ComputeCellCornerPoints(CellNode cell, int robot_radius)
{

    std::deque<Point2D> ceiling, floor;

    for(int i = robot_radius + 1; i < cell.ceiling.size()-(robot_radius + 1); i++)
    {
        if(cell.ceiling[i].y + (robot_radius + 1) <= cell.floor[i].y - (robot_radius + 1))
        {
            ceiling.emplace_back(Point2D(cell.ceiling[i].x, cell.ceiling[i].y + (robot_radius + 1)));
            floor.emplace_back(Point2D(cell.floor[i].x, cell.floor[i].y - (robot_radius + 1)));
        }
    }

    Point2D topleft = ceiling.front();
    Point2D bottomleft = floor.front();
    Point2D bottomright = floor.back();
    Point2D topright = ceiling.back();

    // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
    std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright, topright};

    return corner_points;
}

std::deque<Point2D> GetBoustrophedonPath(std::vector<CellNode>& cell_graph, CellNode cell, int corner_indicator, int robot_radius)
{

    std::deque<Point2D> path;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell, robot_radius);

    std::vector<Point2D> ceiling, floor;
    for(int i = (robot_radius + 1); i < cell.ceiling.size()-(robot_radius + 1); i++)
    {
        if(cell.ceiling[i].y + (robot_radius + 1) <= cell.floor[i].y - (robot_radius + 1))
        {
            ceiling.emplace_back(Point2D(cell.ceiling[i].x, cell.ceiling[i].y + (robot_radius + 1)));
            floor.emplace_back(Point2D(cell.floor[i].x, cell.floor[i].y - (robot_radius + 1)));
        }
    }


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
            int x=0, y=0, y_start=0, y_end=0;
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

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从左往右
                            if( x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+robot_radius+2)].y-floor[i+(j-1+robot_radius+2)].y>=2)
                            &&(i+(j+robot_radius+2) < floor.size())
                            &&(i+(j-1+robot_radius+2) < floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+1)].y-floor[i+(j+2)].y>=2)
                            &&(i+(j+1)< floor.size())
                            &&(i+(j+2)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j-1+robot_radius+1)].y-floor[i+(j+robot_radius+1)].y>=2)
                            &&(i+(j-1+robot_radius+1)< floor.size())
                            &&(i+(j+robot_radius+1)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                            &&(i+(j+1)< floor.size())
                            &&(i+(j)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }

                            path.emplace_back(floor[i+j]);
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
                            if((ceiling[i+(j+robot_radius+1)].y-ceiling[i+(j-1+robot_radius+1)].y>=2)
                            &&(i+(j+robot_radius+1)<ceiling.size())
                            &&(i+(j-1+robot_radius+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                            &&(i+(j)<ceiling.size())
                            &&(i+(j+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j-1+robot_radius+2)].y-ceiling[i+(j+robot_radius+2)].y>=2)
                            &&(i+(j-1+robot_radius+2)<ceiling.size())
                            &&(i+(j+robot_radius+2)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j+2)].y-ceiling[i+(j+1)].y>=2)
                            &&(i+(j+2)<ceiling.size())
                            &&(i+(j+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(ceiling[i+j]);
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

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+1)].y-floor[i-(j+2)].y>=2)
                            &&(i-(j+1)>=0)
                            &&(i-(j+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+robot_radius+2)].y-floor[i-(j-1+robot_radius+2)].y>=2)
                            &&(i-(j+robot_radius+2)>=0)
                            &&(i-(j-1+robot_radius+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                            &&(i-(j+1)>=0)
                            &&(i-(j)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j-1+robot_radius+1)].y-floor[i-(j+robot_radius+1)].y>=2)
                            &&(i-(j-1+robot_radius+1)>=0)
                            &&(i-(j+robot_radius+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(floor[i-j]);
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
                            if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                            &&(i-(j)>=0)
                            &&(i-(j+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j+robot_radius+1)].y-ceiling[i-(j-1+robot_radius+1)].y>=2)
                            &&(i-(j+robot_radius+1)>=0)
                            &&(i-(j-1+robot_radius+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j+2)].y-ceiling[i-(j+1)].y>=2)
                            &&(i-(j+2)>=0)
                            &&(i-(j+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j-1+robot_radius+2)].y-ceiling[i-(j+robot_radius+2)].y>=2)
                            &&(i-(j-1+robot_radius+2)>=0)
                            &&(i-(j+robot_radius+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(ceiling[i-j]);
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
                            if((ceiling[i+(j+robot_radius+1)].y-ceiling[i+(j-1+robot_radius+1)].y>=2)
                               &&(i+(j+robot_radius+1)<ceiling.size())
                               &&(i+(j-1+robot_radius+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j)].y-ceiling[i+(j+1)].y>=2)
                               &&(i+(j)<ceiling.size())
                               &&(i+(j+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j-1+robot_radius+2)].y-ceiling[i+(j+robot_radius+2)].y>=2)
                               &&(i+(j-1+robot_radius+2)<ceiling.size())
                               &&(i+(j+robot_radius+2)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i+(j+2)].y-ceiling[i+(j+1)].y>=2)
                               &&(i+(j+2)<ceiling.size())
                               &&(i+(j+1)<ceiling.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(ceiling[i+j]);
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

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从左往右
                            if(x+j >= ceiling.back().x)
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+robot_radius+2)].y-floor[i+(j-1+robot_radius+2)].y>=2)
                               &&(i+(j+robot_radius+2) < floor.size())
                               &&(i+(j-1+robot_radius+2) < floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+1)].y-floor[i+(j+2)].y>=2)
                               &&(i+(j+1)< floor.size())
                               &&(i+(j+2)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j-1+robot_radius+1)].y-floor[i+(j+robot_radius+1)].y>=2)
                               &&(i+(j-1+robot_radius+1)< floor.size())
                               &&(i+(j+robot_radius+1)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i+(j+1)].y-floor[i+(j)].y>=2)
                               &&(i+(j+1)< floor.size())
                               &&(i+(j)< floor.size()))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(floor[i+j]);
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
                            if((ceiling[i-(j)].y-ceiling[i-(j+1)].y>=2)
                               &&(i-(j)>=0)
                               &&(i-(j+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j+robot_radius+1)].y-ceiling[i-(j-1+robot_radius+1)].y>=2)
                               &&(i-(j+robot_radius+1)>=0)
                               &&(i-(j-1+robot_radius+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j+2)].y-ceiling[i-(j+1)].y>=2)
                               &&(i-(j+2)>=0)
                               &&(i-(j+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((ceiling[i-(j-1+robot_radius+2)].y-ceiling[i-(j+robot_radius+2)].y>=2)
                               &&(i-(j-1+robot_radius+2)>=0)
                               &&(i-(j+robot_radius+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }

                            path.emplace_back(ceiling[i-j]);
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

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            // 沿着floor从右往左
                            if(x-j <= ceiling.front().x)
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+1)].y-floor[i-(j+2)].y>=2)
                               &&(i-(j+1)>=0)
                               &&(i-(j+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+robot_radius+2)].y-floor[i-(j-1+robot_radius+2)].y>=2)
                               &&(i-(j+robot_radius+2)>=0)
                               &&(i-(j-1+robot_radius+2)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j+1)].y-floor[i-(j)].y>=2)
                               &&(i-(j+1)>=0)
                               &&(i-(j)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            if((floor[i-(j-1+robot_radius+1)].y-floor[i-(j+robot_radius+1)].y>=2)
                               &&(i-(j-1+robot_radius+1)>=0)
                               &&(i-(j+robot_radius+1)>=0))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
                            path.emplace_back(floor[i-j]);
                        }
                    }

                    reverse = !reverse;
                }
            }
        }
    }

    return path;
}

bool operator<(const Point2D& p1, const Point2D& p2)
{
    return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
}

bool operator<(const Event& e1, const Event& e2)
{
    return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y) || (e1.x == e2.x && e1.y == e2.y && e1.obstacle_index < e2.obstacle_index));
}

std::vector<Event> InitializeEventList(Polygon polygon, int polygon_index)
{
    std::vector<Event> event_list;
    for(int i = 0; i < polygon.size(); i++)
    {
        event_list.emplace_back(Event(polygon_index, polygon[i].x, polygon[i].y));
    }
    return event_list;
}

//void EventTypeAllocator(cv::Mat map, std::vector<Event>& event_list)
//{
//    int half_size = event_list.size()%2==0? event_list.size()/2 : (event_list.size()+1)/2;
//    std::vector<Event> header(event_list.begin()+half_size, event_list.end());
//    std::vector<Event> tail(event_list.begin(), event_list.begin()+half_size);
//    std::vector<Event> event_list_ex;
//    event_list_ex.insert(event_list_ex.begin(), header.begin(), header.end());
//    event_list_ex.insert(event_list_ex.end(), event_list.begin(), event_list.end());
//    event_list_ex.insert(event_list_ex.end(), tail.begin(), tail.end());
//
//    int index_offset;
//    std::deque<int> in_out_index_list; // 只存放各种in和out的index
//
//    // determine in and out and middle
//    for(int i = half_size; i < half_size + event_list.size(); i++)
//    {
//        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x)
//        {
//            event_list[i-half_size].event_type = IN;
//            in_out_index_list.emplace_back(i-half_size);
//        }
//        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i+1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x < event_list_ex[i+index_offset].x && event_list_ex[i].y < event_list_ex[i+index_offset].y)
//            {
//                event_list[i-half_size].event_type = IN_TOP;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i+1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x < event_list_ex[i+index_offset].x && event_list_ex[i].y > event_list_ex[i+index_offset].y)
//            {
//                event_list[i-half_size].event_type = IN_BOTTOM;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x)
//        {
//            event_list[i-half_size].event_type = MIDDLE;
//        }
//
//        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i-1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x < event_list_ex[i-index_offset].x && event_list_ex[i].y > event_list_ex[i-index_offset].y)
//            {
//                event_list[i-half_size].event_type = IN_BOTTOM;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i-1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x < event_list_ex[i-index_offset].x && event_list_ex[i].y < event_list_ex[i-index_offset].y)
//            {
//                event_list[i-half_size].event_type = IN_TOP;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i-1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x > event_list_ex[i-index_offset].x && event_list_ex[i].y < event_list_ex[i-index_offset].y)
//            {
//                event_list[i-half_size].event_type = OUT_TOP;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i-1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x > event_list_ex[i-index_offset].x && event_list_ex[i].y > event_list_ex[i-index_offset].y)
//            {
//                event_list[i-half_size].event_type = OUT_BOTTOM;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//
//        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x)
//        {
//            event_list[i-half_size].event_type = OUT;
//            in_out_index_list.emplace_back(i-half_size);
//        }
//        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i+1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x > event_list_ex[i+index_offset].x && event_list_ex[i].y > event_list_ex[i+index_offset].y)
//            {
//                event_list[i-half_size].event_type = OUT_BOTTOM;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i+1].y)
//        {
//            index_offset = 2;
//            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
//            {
//                index_offset++;
//            }
//            if(event_list_ex[i].x > event_list_ex[i+index_offset].x && event_list_ex[i].y < event_list_ex[i+index_offset].y)
//            {
//                event_list[i-half_size].event_type = OUT_TOP;
//                in_out_index_list.emplace_back(i-half_size);
//            }
//        }
//
//
//    }
//
//
//    // determine inner
//    Point2D neighbor_point;
//
//    for(int i = 0; i < in_out_index_list.size(); i++)
//    {
//        if(event_list[in_out_index_list[i]].event_type == OUT)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_OUT;
//            }
//        }
//
//        if(event_list[in_out_index_list[i]].event_type == OUT_TOP)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_OUT_TOP;
//            }
//        }
//
//        if(event_list[in_out_index_list[i]].event_type == OUT_BOTTOM)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_OUT_BOTTOM;
//            }
//
//        }
//
//        if(event_list[in_out_index_list[i]].event_type == IN)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_IN;
//            }
//        }
//
//
//        if(event_list[in_out_index_list[i]].event_type == IN_TOP)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_IN_TOP;
//            }
//        }
//
//        if(event_list[in_out_index_list[i]].event_type == IN_BOTTOM)
//        {
//            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
//            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
//            {
//                event_list[in_out_index_list[i]].event_type = INNER_IN_BOTTOM;
//            }
//        }
//    }
//
//    // determine floor and ceiling
//    int temp_index;
//    std::deque<int> ceiling_floor_index_list;
//
//    for(int i = 0; i < in_out_index_list.size(); i++)
//    {
//        if(
//            (event_list[in_out_index_list[0]].event_type==OUT
//            ||event_list[in_out_index_list[0]].event_type==OUT_TOP
//            ||event_list[in_out_index_list[0]].event_type==OUT_BOTTOM
//            ||event_list[in_out_index_list[0]].event_type==INNER_OUT
//            ||event_list[in_out_index_list[0]].event_type==INNER_OUT_TOP
//            ||event_list[in_out_index_list[0]].event_type==INNER_OUT_BOTTOM)
//            &&
//            (event_list[in_out_index_list[1]].event_type==IN
//             ||event_list[in_out_index_list[1]].event_type==IN_TOP
//             ||event_list[in_out_index_list[1]].event_type==IN_BOTTOM
//             ||event_list[in_out_index_list[1]].event_type==INNER_IN
//             ||event_list[in_out_index_list[1]].event_type==INNER_IN_TOP
//             ||event_list[in_out_index_list[1]].event_type==INNER_IN_BOTTOM)
//           )
//        {
//            if(in_out_index_list[0] < in_out_index_list[1])
//            {
//                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
//                {
//                    if(event_list[j].event_type != MIDDLE)
//                    {
//                        event_list[j].event_type = FLOOR;
//                        ceiling_floor_index_list.emplace_back(j);
//                    }
//                }
//            }
//            else
//            {
//                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
//                {
//                    if(event_list[j].event_type != MIDDLE)
//                    {
//                        event_list[j].event_type = FLOOR;
//                        ceiling_floor_index_list.emplace_back(j);
//                    }
//                }
//                for(int k = 0; k < in_out_index_list[1]; k++)
//                {
//                    if(event_list[k].event_type != MIDDLE)
//                    {
//                        event_list[k].event_type = FLOOR;
//                        ceiling_floor_index_list.emplace_back(k);
//                    }
//                }
//            }
//        }
//
//        if(
//             (event_list[in_out_index_list[0]].event_type==IN
//             ||event_list[in_out_index_list[0]].event_type==IN_TOP
//             ||event_list[in_out_index_list[0]].event_type==IN_BOTTOM
//             ||event_list[in_out_index_list[0]].event_type==INNER_IN
//             ||event_list[in_out_index_list[0]].event_type==INNER_IN_TOP
//             ||event_list[in_out_index_list[0]].event_type==INNER_IN_BOTTOM)
//             &&
//             (event_list[in_out_index_list[1]].event_type==OUT
//             ||event_list[in_out_index_list[1]].event_type==OUT_TOP
//             ||event_list[in_out_index_list[1]].event_type==OUT_BOTTOM
//             ||event_list[in_out_index_list[1]].event_type==INNER_OUT
//             ||event_list[in_out_index_list[1]].event_type==INNER_OUT_TOP
//             ||event_list[in_out_index_list[1]].event_type==INNER_OUT_BOTTOM)
//           )
//        {
//            if(in_out_index_list[0] < in_out_index_list[1])
//            {
//                for(int j = in_out_index_list[0]+1; j < in_out_index_list[1]; j++)
//                {
//                    if(event_list[j].event_type != MIDDLE)
//                    {
//                        event_list[j].event_type = CEILING;
//                        ceiling_floor_index_list.emplace_back(j);
//                    }
//                }
//            }
//            else
//            {
//                for(int j = in_out_index_list[0]+1; j < event_list.size(); j++)
//                {
//                    if(event_list[j].event_type != MIDDLE)
//                    {
//                        event_list[j].event_type = CEILING;
//                        ceiling_floor_index_list.emplace_back(j);
//                    }
//                }
//                for(int k = 0; k < in_out_index_list[1]; k++)
//                {
//                    if(event_list[k].event_type != MIDDLE)
//                    {
//                        event_list[k].event_type = CEILING;
//                        ceiling_floor_index_list.emplace_back(k);
//                    }
//                }
//            }
//        }
//
//        temp_index = in_out_index_list.front();
//        in_out_index_list.pop_front();
//        in_out_index_list.emplace_back(temp_index);
//    }
//
//
//    // filter ceiling and floor
//    for(int i = 0; i < ceiling_floor_index_list.size()-1; i++)
//    {
//        if(event_list[ceiling_floor_index_list[i]].event_type==CEILING
//        && event_list[ceiling_floor_index_list[i+1]].event_type==CEILING
//        && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
//        {
//            if(event_list[ceiling_floor_index_list[i]].y>event_list[ceiling_floor_index_list[i+1]].y)
//            {
//                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
//            }
//            else
//            {
//                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
//            }
//        }
//        if(event_list[ceiling_floor_index_list[i]].event_type==FLOOR
//        && event_list[ceiling_floor_index_list[i+1]].event_type==FLOOR
//        && event_list[ceiling_floor_index_list[i]].x==event_list[ceiling_floor_index_list[i+1]].x)
//        {
//            if(event_list[ceiling_floor_index_list[i]].y<event_list[ceiling_floor_index_list[i+1]].y)
//            {
//                event_list[ceiling_floor_index_list[i+1]].event_type = MIDDLE;
//            }
//            else
//            {
//                event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
//            }
//        }
//    }
//    if(event_list[ceiling_floor_index_list.back()].event_type==CEILING
//    && event_list[ceiling_floor_index_list.front()].event_type==CEILING
//    && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
//    {
//        if(event_list[ceiling_floor_index_list.back()].y>event_list[ceiling_floor_index_list.front()].y)
//        {
//            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
//        }
//        else
//        {
//            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
//        }
//    }
//    if(event_list[ceiling_floor_index_list.back()].event_type==FLOOR
//    && event_list[ceiling_floor_index_list.front()].event_type==FLOOR
//    && event_list[ceiling_floor_index_list.back()].x==event_list[ceiling_floor_index_list.front()].x)
//    {
//        if(event_list[ceiling_floor_index_list.back()].y<event_list[ceiling_floor_index_list.front()].y)
//        {
//            event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
//        }
//        else
//        {
//            event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
//        }
//    }
//} //旧版

void EventTypeAllocator2(const cv::Mat& map, std::vector<Event>& event_list, int robot_radius)
{
    int half_size = event_list.size()%2==0? event_list.size()/2 : (event_list.size()+1)/2;
    std::vector<Event> header(event_list.begin()+half_size, event_list.end());
    std::vector<Event> tail(event_list.begin(), event_list.begin()+half_size);
    std::vector<Event> event_list_ex;
    event_list_ex.insert(event_list_ex.begin(), header.begin(), header.end());
    event_list_ex.insert(event_list_ex.end(), event_list.begin(), event_list.end());
    event_list_ex.insert(event_list_ex.end(), tail.begin(), tail.end());

    int index_offset;
    std::deque<int> in_out_index_list; // 只存放各种in和out的index

    // determine in and out and middle
    for(int i = half_size; i < half_size + event_list.size(); i++)
    {
        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x)
        {
            event_list[i-half_size].event_type = IN;
            in_out_index_list.emplace_back(i-half_size);
        }
        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i+1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x < event_list_ex[i+index_offset].x && event_list_ex[i].y < event_list_ex[i+index_offset].y)
            {
                event_list[i-half_size].event_type = IN_TOP;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i+1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x < event_list_ex[i+index_offset].x && event_list_ex[i].y > event_list_ex[i+index_offset].y)
            {
                event_list[i-half_size].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x)
        {
            event_list[i-half_size].event_type = MIDDLE;
        }

        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i-1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x < event_list_ex[i-index_offset].x && event_list_ex[i].y > event_list_ex[i-index_offset].y)
            {
                event_list[i-half_size].event_type = IN_BOTTOM;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i-1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x < event_list_ex[i-index_offset].x && event_list_ex[i].y < event_list_ex[i-index_offset].y)
            {
                event_list[i-half_size].event_type = IN_TOP;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i-1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x > event_list_ex[i-index_offset].x && event_list_ex[i].y < event_list_ex[i-index_offset].y)
            {
                event_list[i-half_size].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x == event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i-1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i-index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x > event_list_ex[i-index_offset].x && event_list_ex[i].y > event_list_ex[i-index_offset].y)
            {
                event_list[i-half_size].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i-half_size);
            }
        }


        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x)
        {
            event_list[i-half_size].event_type = OUT;
            in_out_index_list.emplace_back(i-half_size);
        }
        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y > event_list_ex[i+1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x > event_list_ex[i+index_offset].x && event_list_ex[i].y > event_list_ex[i+index_offset].y)
            {
                event_list[i-half_size].event_type = OUT_BOTTOM;
                in_out_index_list.emplace_back(i-half_size);
            }
        }

        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x == event_list_ex[i+1].x && event_list_ex[i].y < event_list_ex[i+1].y)
        {
            index_offset = 2;
            while(event_list_ex[i].x == event_list_ex[i+index_offset].x)
            {
                index_offset++;
            }
            if(event_list_ex[i].x > event_list_ex[i+index_offset].x && event_list_ex[i].y < event_list_ex[i+index_offset].y)
            {
                event_list[i-half_size].event_type = OUT_TOP;
                in_out_index_list.emplace_back(i-half_size);
            }
        }


    }

    // filter top and bottom
    int center_index;
    int temp_index;
    int in_out_index_list_size = in_out_index_list.size();
    for(int i = 1; i <= in_out_index_list_size; i++)
    {
        if(event_list[in_out_index_list[0]].event_type == IN_TOP
        && event_list[in_out_index_list[1]].event_type == IN_BOTTOM
        && abs(event_list[in_out_index_list[0]].y-event_list[in_out_index_list[1]].y)<=robot_radius)
        {
            center_index = (in_out_index_list[0]+in_out_index_list[1])%2==0? (in_out_index_list[0]+in_out_index_list[1])/2 : (in_out_index_list[0]+in_out_index_list[1]+1)/2;
            event_list[in_out_index_list[0]].event_type = MIDDLE;
            event_list[in_out_index_list[1]].event_type = MIDDLE;
            event_list[center_index].event_type = IN;
            in_out_index_list.pop_front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_front(center_index);
        }

        if(event_list[in_out_index_list[0]].event_type == IN_BOTTOM
           && event_list[in_out_index_list[1]].event_type == IN_TOP
           && abs(event_list[in_out_index_list[0]].y-event_list[in_out_index_list[1]].y)<=robot_radius)
        {
            center_index = (in_out_index_list[0]+in_out_index_list[1])%2==0? (in_out_index_list[0]+in_out_index_list[1])/2 : (in_out_index_list[0]+in_out_index_list[1]+1)/2;
            event_list[in_out_index_list[0]].event_type = MIDDLE;
            event_list[in_out_index_list[1]].event_type = MIDDLE;
            event_list[center_index].event_type = IN;
            in_out_index_list.pop_front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_front(center_index);
        }

        if(event_list[in_out_index_list[0]].event_type == OUT_TOP
           && event_list[in_out_index_list[1]].event_type == OUT_BOTTOM
           && abs(event_list[in_out_index_list[0]].y-event_list[in_out_index_list[1]].y)<=robot_radius)
        {
            center_index = (in_out_index_list[0]+in_out_index_list[1])%2==0? (in_out_index_list[0]+in_out_index_list[1])/2 : (in_out_index_list[0]+in_out_index_list[1]+1)/2;
            event_list[in_out_index_list[0]].event_type = MIDDLE;
            event_list[in_out_index_list[1]].event_type = MIDDLE;
            event_list[center_index].event_type = OUT;
            in_out_index_list.pop_front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_front(center_index);
        }

        if(event_list[in_out_index_list[0]].event_type == OUT_BOTTOM
           && event_list[in_out_index_list[1]].event_type == OUT_TOP
           && abs(event_list[in_out_index_list[0]].y-event_list[in_out_index_list[1]].y)<=robot_radius)
        {
            center_index = (in_out_index_list[0]+in_out_index_list[1])%2==0? (in_out_index_list[0]+in_out_index_list[1])/2 : (in_out_index_list[0]+in_out_index_list[1]+1)/2;
            event_list[in_out_index_list[0]].event_type = MIDDLE;
            event_list[in_out_index_list[1]].event_type = MIDDLE;
            event_list[center_index].event_type = OUT;
            in_out_index_list.pop_front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_front(center_index);
        }
        temp_index = in_out_index_list.front();
        in_out_index_list.pop_front();
        in_out_index_list.emplace_back(temp_index);
    }

    // determine inner
    Point2D neighbor_point;

    for(int i = 0; i < in_out_index_list.size(); i++)
    {
        if(event_list[in_out_index_list[i]].event_type == OUT)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_OUT;
            }
        }

        if(event_list[in_out_index_list[i]].event_type == OUT_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_OUT_TOP;
            }
        }

        if(event_list[in_out_index_list[i]].event_type == OUT_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x+1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_OUT_BOTTOM;
            }

        }

        if(event_list[in_out_index_list[i]].event_type == IN)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_IN;
            }
        }


        if(event_list[in_out_index_list[i]].event_type == IN_TOP)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_IN_TOP;
            }
        }

        if(event_list[in_out_index_list[i]].event_type == IN_BOTTOM)
        {
            neighbor_point = Point2D(event_list[in_out_index_list[i]].x-1, event_list[in_out_index_list[i]].y);
            if(map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255,255,255))
            {
                event_list[in_out_index_list[i]].event_type = INNER_IN_BOTTOM;
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

// 各个多边形按照其左顶点的x值从小到大的顺序进行排列
std::vector<Event> EventListGenerator(const cv::Mat& map, PolygonList polygons, int robot_radius)
{
    std::vector<Event> event_list;
    std::vector<Event> event_sublist;

    for(int i = 0; i < polygons.size(); i++)
    {
        event_sublist = InitializeEventList(polygons[i], i);
        EventTypeAllocator2(map, event_sublist, robot_radius);
        event_list.insert(event_list.end(), event_sublist.begin(), event_sublist.end());
        event_sublist.clear();
    }

    std::sort(event_list.begin(), event_list.end());

    return event_list;
}

std::deque<std::deque<Event>> SliceListGenerator(std::vector<Event> event_list)
{
    std::deque<std::deque<Event>> slice_list;
    std::deque<Event> slice;
    int x = event_list.front().x;

    for(int i = 0; i < event_list.size(); i++)
    {
        if(event_list[i].x != x)
        {
            slice_list.emplace_back(slice);

            x = event_list[i].x;
            slice.clear();
            slice.emplace_back(event_list[i]);
        }
        else
        {
            slice.emplace_back(event_list[i]);
        }
    }
    slice_list.emplace_back(slice);

    return slice_list;
}

void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false) // in event  add two new node
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

void ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D out, Point2D c, Point2D f, bool rewrite = false) // out event  add one new node
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

void ExecuteCeilOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& ceil_point) // finish constructing last ceiling edge
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void ExecuteFloorOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, const Point2D& floor_point) // finish constructing last floor edge
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}

void ExecuteOpenOperation(std::vector<CellNode>& cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false) // in event  add two new node
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

void ExecuteCloseOperation(std::vector<CellNode>& cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D out_top, Point2D out_bottom, Point2D c, Point2D f, bool rewrite = false) // out event  add one new node
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

void InitializeCellDecomposition(const cv::Mat& map, std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, Point2D first_in_pos)
{
    CellNode cell_0;

    int cell_0_idx = 0;
    cell_0.cellIndex = cell_0_idx;

    for(int i = 0; i < first_in_pos.x; i++)
    {
        cell_0.ceiling.emplace_back(Point2D(i,0));
        cell_0.floor.emplace_back(Point2D(i,map.rows-1));
    }

    cell_graph.emplace_back(cell_0);
    cell_index_slice.emplace_back(0);
}//旧版

void InitializeCellDecomposition2(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, Point2D first_in_pos, CellNode outermost_cell)
{
    CellNode cell_0;

    int cell_0_idx = 0;
    cell_0.cellIndex = cell_0_idx;

    int index = 0;
    while(outermost_cell.ceiling[index].x < first_in_pos.x)
    {
        cell_0.ceiling.emplace_back(outermost_cell.ceiling[index]);
        cell_0.floor.emplace_back(outermost_cell.floor[index]);
        index++;
    }

    cell_graph.emplace_back(cell_0);
    cell_index_slice.emplace_back(0);
}

void FinishCellDecomposition(const cv::Mat& map, std::vector<CellNode>& cell_graph, Point2D last_out_pos)
{
    int last_cell_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点
    for(int i = last_out_pos.x + 1; i <= map.cols-1; i++)
    {
        cell_graph[last_cell_idx].ceiling.emplace_back(Point2D(i, 0));
        cell_graph[last_cell_idx].floor.emplace_back(Point2D(i, map.rows-1));
    }
}//旧版

void FinishCellDecomposition2(std::vector<CellNode>& cell_graph, Point2D last_out_pos, CellNode outermost_cell)
{
    int last_cell_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点
    int index = outermost_cell.ceiling.size()-1;
    while(outermost_cell.ceiling[index].x >= (last_out_pos.x + 1))
    {
        cell_graph[last_cell_idx].ceiling.emplace_front(outermost_cell.ceiling[index]);
        cell_graph[last_cell_idx].floor.emplace_front(outermost_cell.floor[index]);
        index--;
    }
}

void DrawCells(cv::Mat& map, const CellNode& cell)
{
    std::cout<<"cell "<<cell.cellIndex<<": "<<std::endl;
    std::cout<<"cell's ceiling points: "<<cell.ceiling.size()<<std::endl;
    std::cout<<"cell's floor points: "<<cell.floor.size()<<std::endl;

    for(int i = 0; i < cell.ceiling.size(); i++)
    {
        map.at<cv::Vec3b>(cell.ceiling[i].y, cell.ceiling[i].x) = cv::Vec3b(0, 0, 255); // 96 96 96
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        map.at<cv::Vec3b>(cell.floor[i].y, cell.floor[i].x) = cv::Vec3b(0, 0, 255);
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), cv::Scalar(0, 0, 255));
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), cv::Scalar(0, 0, 255));
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
          )
        {
            cell_num++;
        }
    }
    return cell_num;
}

std::deque<Event> FilterSlice(std::deque<Event> slice)
{
    std::deque<Event> filtered_slice;

    for(int i = 0; i < slice.size(); i++)
    {
        if(slice[i].event_type!=MIDDLE && slice[i].event_type!=UNALLOCATED)
        {
            filtered_slice.emplace_back(slice[i]);
        }
    }
    return filtered_slice;
}

void ExecuteCellDecomposition(const cv::Mat& map, std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, std::deque<std::deque<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    Point2D c, f;
    int c_index = INT_MAX, f_index = INT_MAX;
    int min_dist = INT_MAX;

    int slice_x = INT_MAX;
    int event_y = INT_MAX;

    bool rewrite = false;

    std::vector<int> sub_cell_index_slices;
    std::deque<Event> curr_slice;

    int cell_counter = 0;

    for(int i = 0; i < slice_list.size(); i++)
    {
        slice_x = slice_list[i].front().x;

        curr_slice = FilterSlice(slice_list[i]);

        curr_slice.emplace_front(Event(INT_MAX, slice_x, 0, CEILING));       // add map upper boundary
        curr_slice.emplace_back(Event(INT_MAX, slice_x, map.rows-1, FLOOR)); // add map lower boundary

        original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());

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
                                              Point2D(curr_slice[j].x, curr_slice[j].y),
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
                                              Point2D(curr_slice[j-1].x, curr_slice[j-1].y),   // out top
                                              Point2D(curr_slice[j].x, curr_slice[j].y),       // out bottom

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
                for(int k = 1; k < cell_index_slice.size(); k++)
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
                for(int k = 1; k < cell_index_slice.size(); k++)
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
}//旧版

void ExecuteCellDecomposition2(std::vector<CellNode>& cell_graph, std::vector<int>& cell_index_slice, std::vector<int>& original_cell_index_slice, std::deque<std::deque<Event>> slice_list, CellNode outermost_cell)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    Point2D c, f;
    int c_index = INT_MAX, f_index = INT_MAX;
    int min_dist = INT_MAX;

    int slice_x = INT_MAX;
    int event_y = INT_MAX;

    bool rewrite = false;

    std::vector<int> sub_cell_index_slices;
    std::deque<Event> curr_slice;

    int cell_counter = 0;

    int outermost_cell_index_offset;

    for(int i = 0; i < slice_list.size(); i++)
    {
        slice_x = slice_list[i].front().x;

        curr_slice = FilterSlice(slice_list[i]);

        outermost_cell_index_offset = slice_x - outermost_cell.ceiling.front().x;
        curr_slice.emplace_front(Event(INT_MAX, slice_x, outermost_cell.ceiling[outermost_cell_index_offset].y, CEILING));       // add map upper boundary
        curr_slice.emplace_back(Event(INT_MAX, slice_x, outermost_cell.floor[outermost_cell_index_offset].y, FLOOR)); // add map lower boundary

        original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());

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
                                              Point2D(curr_slice[j].x, curr_slice[j].y),
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
                                              Point2D(curr_slice[j-1].x, curr_slice[j-1].y),   // out top
                                              Point2D(curr_slice[j].x, curr_slice[j].y),       // out bottom

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
                for(int k = 1; k < cell_index_slice.size(); k++)
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
                for(int k = 1; k < cell_index_slice.size(); k++)
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

Point2D FindNextEntrance(Point2D curr_point, CellNode next_cell, int& corner_indicator, int robot_radius)
{
    Point2D next_entrance = Point2D(INT_MAX, INT_MAX);

    int front_x = next_cell.ceiling.front().x;
    int back_x = next_cell.ceiling.back().x;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(next_cell, robot_radius);

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

std::deque<Point2D> ExitAlongWall(Point2D start, Point2D& end, CellNode cell, int robot_radius)
{
    int start_corner_indicator = INT_MAX;
    int end_corner_indicator = INT_MAX;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell, robot_radius);
    for(int i = 0; i < corner_points.size(); i++)
    {
        if(corner_points[i].x==start.x && corner_points[i].y == start.y)
        {
            start_corner_indicator = i;
        }
        if(corner_points[i].x==end.x && corner_points[i].y == end.y)
        {
            end_corner_indicator = i;
        }
    }

    std::vector<Point2D> top, bottom;  // 从左往右

    for(int i = robot_radius+1; i < cell.ceiling.size()-(robot_radius+1); i++)
    {
        if(cell.ceiling[i].y + (robot_radius + 1) <= cell.floor[i].y - (robot_radius + 1))
        {
            top.emplace_back(Point2D(cell.ceiling[i].x, cell.ceiling[i].y + (robot_radius + 1)));
            bottom.emplace_back(Point2D(cell.floor[i].x, cell.floor[i].y - (robot_radius + 1)));
        }
    }

    std::deque<Point2D> path;
    std::deque<Point2D> temp_path;

    if (start_corner_indicator == end_corner_indicator)
    {
        return path;
    }

    if(start_corner_indicator == TOPLEFT && end_corner_indicator == TOPRIGHT)
    {
        temp_path.assign(top.begin(), top.end());
        for(int i = 0; i < temp_path.size(); i++)
        {
            // 提前转
            if((temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y>=2)&&(i+robot_radius+1<temp_path.size())&&(i-1+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y+abs(delta)));
                }
                i += (robot_radius+1);
            }
            // 滞后转
            if((temp_path[i].y-temp_path[i+1].y>=2)&&(i<temp_path.size())&&(i+1<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y+abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y+ + abs(delta) + increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMLEFT)
    {
        end.x = corner_points[TOPLEFT].x;
        end.y = corner_points[TOPLEFT].y;
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        temp_path.assign(top.begin(), top.end());
        for(int i = 0; i < temp_path.size(); i++)
        {
            // 提前转
            if((temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y>=2)&&(i+robot_radius+1<temp_path.size())&&(i-1+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y+abs(delta)));
                }
                i += (robot_radius+1);
            }
            // 滞后转
            if((temp_path[i].y-temp_path[i+1].y>=2)&&(i<temp_path.size())&&(i+1<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y+abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y+ abs(delta) +increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        end.x = corner_points[TOPRIGHT].x;
        end.y = corner_points[TOPRIGHT].y;
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == TOPLEFT)
    {
        temp_path.assign(top.rbegin(), top.rend());
        for(int i = 0; i < temp_path.size(); i++)
        {
            // 提前转
            if((temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y>=2)&&(i+robot_radius+1<temp_path.size())&&(i-1+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y+abs(delta)));
                }
                i += (robot_radius+1);
            }
            // 滞后转
            if((temp_path[i].y-temp_path[i+1].y>=2)&&(i<temp_path.size())&&(i+1<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y+abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y+ abs(delta) +increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        temp_path.assign(top.rbegin(), top.rend());
        for(int i = 0; i < temp_path.size(); i++)
        {
            // 提前转
            if((temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y>=2)&&(i+robot_radius+1<temp_path.size())&&(i-1+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y+abs(delta)));
                }
                i += (robot_radius+1);
            }
            // 滞后转
            if((temp_path[i].y-temp_path[i+1].y>=2)&&(i<temp_path.size())&&(i+1<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y+abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y+ abs(delta) +increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        end.x = corner_points[TOPLEFT].x;
        end.y = corner_points[TOPLEFT].y;
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMRIGHT)
    {
        end.x = corner_points[TOPRIGHT].x;
        end.y = corner_points[TOPRIGHT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPLEFT)
    {
        end.x = corner_points[BOTTOMLEFT].x;
        end.y = corner_points[BOTTOMLEFT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPRIGHT)
    {
        temp_path.assign(bottom.begin(), bottom.end());
        for(int i = 0; i < temp_path.size(); i++)
        {
            //提前转
            if((temp_path[i-1+robot_radius+1].y-temp_path[i+robot_radius+1].y>=2)&&(i-1+robot_radius+1<temp_path.size())&&(i+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);

                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }

                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y-abs(delta)));
                }
                i+= (robot_radius+1);
            }
            //滞后转
            if((temp_path[i+1].y-temp_path[i].y>=2)&&(i+1<temp_path.size())&&(i<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y-abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y -abs(delta) + increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        end.x = corner_points[BOTTOMRIGHT].x;
        end.y = corner_points[BOTTOMRIGHT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        temp_path.assign(bottom.begin(), bottom.end());
        for(int i = 0; i < temp_path.size(); i++)
        {
            //提前转
            if((temp_path[i-1+robot_radius+1].y-temp_path[i+robot_radius+1].y>=2)&&(i-1+robot_radius+1<temp_path.size())&&(i+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);

                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }

                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y-abs(delta)));
                }
                i+= (robot_radius+1);
            }
            //滞后转
            if((temp_path[i+1].y-temp_path[i].y>=2)&&(i+1<temp_path.size())&&(i<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y-abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y -abs(delta) + increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }


        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPLEFT)
    {
        temp_path.assign(bottom.rbegin(), bottom.rend());
        for(int i = 0; i < temp_path.size(); i++)
        {
            //提前转
            if((temp_path[i-1+robot_radius+1].y-temp_path[i+robot_radius+1].y>=2)&&(i-1+robot_radius+1<temp_path.size())&&(i+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);

                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }

                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y-abs(delta)));
                }
                i+= (robot_radius+1);
            }
            //滞后转
            if((temp_path[i+1].y-temp_path[i].y>=2)&&(i+1<temp_path.size())&&(i<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y-abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y -abs(delta) + increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        end.x = corner_points[BOTTOMLEFT].x;
        end.y = corner_points[BOTTOMLEFT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPRIGHT)
    {
        end.x = corner_points[BOTTOMRIGHT].x;
        end.y = corner_points[BOTTOMRIGHT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        temp_path.assign(bottom.rbegin(), bottom.rend());
        for(int i = 0; i < temp_path.size(); i++)
        {
            //提前转
            if((temp_path[i-1+robot_radius+1].y-temp_path[i+robot_radius+1].y>=2)&&(i-1+robot_radius+1<temp_path.size())&&(i+robot_radius+1<temp_path.size()))
            {
                int delta = temp_path[i+robot_radius+1].y-temp_path[i-1+robot_radius+1].y;
                int increment = delta/abs(delta);

                for(int j = 0; j <= abs(delta); j++)
                {
                    path.emplace_back(Point2D(temp_path[i].x, temp_path[i].y+increment*j));
                }

                for(int k = 1; k <= robot_radius; k++)
                {
                    path.emplace_back(Point2D(temp_path[i+k].x, temp_path[i+k].y-abs(delta)));
                }
                i+= (robot_radius+1);
            }
            //滞后转
            if((temp_path[i+1].y-temp_path[i].y>=2)&&(i+1<temp_path.size())&&(i<temp_path.size()))
            {
                path.emplace_back(temp_path[i]);

                int delta = temp_path[i+1].y-temp_path[i].y;
                int increment = delta/abs(delta);
                for(int j = 1; j <= robot_radius; j++)
                {
                    path.emplace_back(Point2D(temp_path[i+j].x, temp_path[i+j].y-abs(delta)));
                }
                for(int k = 0; k <= abs(delta); k++)
                {
                    path.emplace_back(Point2D(temp_path[i+robot_radius+1].x, temp_path[i+robot_radius+1].y -abs(delta) + increment*k));
                }
                i += (robot_radius+2);
            }
            path.emplace_back(temp_path[i]);
        }
        return path;
    }

}

std::deque<Point2D> FindLinkingPath(Point2D curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, CellNode next_cell, int robot_radius)
{
    std::deque<Point2D> path;
    std::deque<Point2D> sub_path;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator, robot_radius);
    sub_path = ExitAlongWall(curr_exit, exit, curr_cell, robot_radius);
    path.insert(path.begin(), sub_path.begin(), sub_path.end());

    next_entrance = FindNextEntrance(exit, next_cell, corner_indicator, robot_radius);

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

    if (exit.x >= curr_cell.ceiling.back().x - (robot_radius + 1))
    {
        upper_bound = (curr_cell.ceiling.end() - 1 - (robot_radius + 1))->y;
        lower_bound = (curr_cell.floor.end() - 1 - (robot_radius + 1))->y;
    }
    if (exit.x <= curr_cell.ceiling.front().x + (robot_radius + 1))
    {
        upper_bound = (curr_cell.ceiling.begin() + (robot_radius + 1))->y;
        lower_bound = (curr_cell.floor.begin() + (robot_radius + 1))->y;
    }

    if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound)) {
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path.emplace_back(Point2D(exit.x, y));
        }
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path.emplace_back(Point2D(x, next_entrance.y));
        }
    } else {
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path.emplace_back(Point2D(x, exit.y));
        }
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path.emplace_back(Point2D(next_entrance.x, y));
        }
    }
//    path.emplace_back(next_entrance);

    return path;
}

std::deque<Point2D> PathIninitialization(Point2D start, CellNode cell, int robot_radius)
{
    std::deque<Point2D> path;

    int index_offset = std::abs(start.x - cell.ceiling.front().x);

    for(int y = start.y; y >= cell.ceiling[index_offset].y+(robot_radius+1); y--)
    {
        path.emplace_back(Point2D(start.x, y));
    }

    for(int i = index_offset; i >= robot_radius+1; i--)
    {
        path.emplace_back(Point2D(cell.ceiling[i].x,cell.ceiling[i].y+(robot_radius+1)));
    }

    return path;
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

int DetermineCellIndex(std::vector<CellNode>& cell_graph, Point2D point)
{
    int cell_index;

    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].ceiling.size(); j++)
        {
            if(point.x ==  cell_graph[i].ceiling[j].x && point.y > cell_graph[i].ceiling[j].y && point.y < cell_graph[i].floor[j].y)
            {
                cell_index = i;
                return cell_index;
            }
        }

    }
}

std::deque<int> FindShortestPath(std::vector<CellNode>& cell_graph, Point2D start, Point2D end)
{
    int start_cell_index = DetermineCellIndex(cell_graph, start);
    int end_cell_index = DetermineCellIndex(cell_graph, end);

    std::deque<int> cell_path = {end_cell_index};
    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());

    for(int i = 0; i < cells.size(); i++)
    {
        cells[i].isVisited = false;
        cells[i].isCleaned = false;
        cells[i].parentIndex = INT_MAX;
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
} // BFS

std::deque<Point2D> WalkingInsideCell(CellNode cell, Point2D start, Point2D end, int robot_radius)
{
    std::deque<Point2D> inner_path = {start};

    int start_ceiling_index_offset = start.x - cell.ceiling.front().x;
    int first_ceiling_delta_y = cell.ceiling[start_ceiling_index_offset].y + (robot_radius + 1) - start.y;
    int end_ceiling_index_offset = end.x - cell.ceiling.front().x;
    int second_ceiling_delta_y = end.y - (cell.ceiling[end_ceiling_index_offset].y + (robot_radius + 1));

    int start_floor_index_offset = start.x - cell.floor.front().x;
    int first_floor_delta_y = cell.floor[start_floor_index_offset].y - (robot_radius + 1) - start.y;
    int end_floor_index_offset = end.x - cell.floor.front().x;
    int second_floor_delta_y = end.y - (cell.floor[end_floor_index_offset].y - (robot_radius + 1));

    if((abs(first_ceiling_delta_y)+abs(second_ceiling_delta_y)) < (abs(first_floor_delta_y)+abs(second_floor_delta_y))) //to ceiling
    {
        int first_increment_y = 0;
        if(first_ceiling_delta_y != 0)
        {
            first_increment_y = first_ceiling_delta_y / abs(first_ceiling_delta_y);
        }
        for(int i = 1; i <= abs(first_ceiling_delta_y); i++)
        {
            inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
        }

        int delta_x = cell.ceiling[end_ceiling_index_offset].x - cell.ceiling[start_ceiling_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 1; i <= abs(delta_x); i++)
        {
            // 提前转
            if((cell.ceiling[start_ceiling_index_offset+increment_x*(i+robot_radius+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i-1+robot_radius+1)].y>=2)
            &&(i+robot_radius+1 <= abs(delta_x))
            &&(i-1+robot_radius+1 <= abs(delta_x)))
            {
                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+robot_radius+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i-1+robot_radius+1)].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*i].x, cell.ceiling[start_ceiling_index_offset+increment_x*i].y+(robot_radius + 1)+increment*(j)));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+k)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+k)].y+(robot_radius + 1)+abs(delta)));
                }
                i += (robot_radius+1);
            }
            // 滞后转
            if((cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y>=2)
            &&(i<=abs(delta_x))
            &&(i+1<=abs(delta_x)))
            {
                inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y+(robot_radius+1)));

                int delta = cell.ceiling[start_ceiling_index_offset+increment_x*(i+1)].y-cell.ceiling[start_ceiling_index_offset+increment_x*(i)].y;
                for(int j = 1; j<= robot_radius; j++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+j)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+j)].y+(robot_radius+1)+abs(delta)));
                }
                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+increment_x*(i+robot_radius+1)].x, cell.ceiling[start_ceiling_index_offset+increment_x*(i+robot_radius+1)].y+(robot_radius+1)+abs(delta)+increment*(k)));
                }
                i += (robot_radius+2);
            }
            inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset+(increment_x*i)].x,cell.ceiling[start_ceiling_index_offset+(increment_x*i)].y+(robot_radius + 1)));
        }

        int second_increment_y = 0;
        if(second_ceiling_delta_y!=0)
        {
            second_increment_y = second_ceiling_delta_y/abs(second_ceiling_delta_y);
        }
        for(int i = 1; i < abs(second_ceiling_delta_y); i++)
        {
            inner_path.emplace_back(Point2D(cell.ceiling[end_ceiling_index_offset].x, cell.ceiling[end_ceiling_index_offset].y+(robot_radius + 1)+(second_increment_y*i)));
        }
    }
    else // to floor
    {
        int first_increment_y = 0;
        if(first_floor_delta_y != 0)
        {
            first_increment_y = first_floor_delta_y / abs(first_floor_delta_y);
        }
        for(int i = 1; i <= abs(first_floor_delta_y); i++)
        {
            inner_path.emplace_back(Point2D(start.x, start.y+(first_increment_y*i)));
        }

        int delta_x = cell.floor[end_floor_index_offset].x - cell.floor[start_floor_index_offset].x;
        int increment_x = 0;
        if(delta_x != 0)
        {
            increment_x = delta_x / abs(delta_x);
        }
        for(int i = 1; i <= abs(delta_x); i++)
        {
            //提前转
            if((cell.floor[start_floor_index_offset+increment_x*(i-1+(robot_radius+1))].y-cell.floor[start_floor_index_offset+increment_x*(i+(robot_radius+1))].y>=2)
            &&(i-1+(robot_radius+1)<=abs(delta_x))
            &&(i+(robot_radius+1)<=abs(delta_x)))
            {
                int delta = cell.floor[start_floor_index_offset+increment_x*(i+(robot_radius+1))].y-cell.floor[start_floor_index_offset+increment_x*(i-1+(robot_radius+1))].y;
                int increment = delta/abs(delta);
                for(int j = 0; j <= abs(delta); j++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y-(robot_radius+1)+increment*(j)));
                }
                for(int k = 1; k <= robot_radius; k++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+k)].x, cell.floor[start_floor_index_offset+increment_x*(i+k)].y-(robot_radius+1)-abs(delta)));
                }
                i += (robot_radius + 1);
            }
            //滞后转
            if((cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y>=2)
            &&(i+1<=abs(delta_x))
            &&(i<=abs(delta_x)))
            {
                inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i)].x, cell.floor[start_floor_index_offset+increment_x*(i)].y-(robot_radius+1)));

                int delta = cell.floor[start_floor_index_offset+increment_x*(i+1)].y-cell.floor[start_floor_index_offset+increment_x*(i)].y;
                for(int j = 1; j <= robot_radius; j++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+j)].x, cell.floor[start_floor_index_offset+increment_x*(i+j)].y-(robot_radius+1)-abs(delta)));
                }
                int increment = delta/abs(delta);
                for(int k = 0; k <= abs(delta); k++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+increment_x*(i+(robot_radius+1))].x, cell.floor[start_floor_index_offset+increment_x*(i+(robot_radius+1))].y-(robot_radius+1) -abs(delta) +increment*(k)));
                }
                i += (robot_radius+2);
            }
            inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset+(increment_x*i)].x,cell.floor[start_floor_index_offset+(increment_x*i)].y-(robot_radius + 1)));
        }

        int second_increment_y = 0;
        if(second_floor_delta_y!=0)
        {
            second_increment_y = second_floor_delta_y/abs(second_floor_delta_y);
        }
        for(int i = 1; i < abs(second_floor_delta_y); i++)
        {
            inner_path.emplace_back(Point2D(cell.floor[end_floor_index_offset].x, cell.floor[end_floor_index_offset].y-(robot_radius + 1)+(second_increment_y*i)));
        }
    }
    return inner_path;
}

std::deque<Point2D> WalkingCrossCells(std::vector<CellNode>& cell_graph, std::deque<int> cell_path, Point2D start, Point2D end, int robot_radius)
{
    std::deque<Point2D> overall_path;
    std::deque<Point2D> sub_path;

    std::vector<CellNode> cells;
    cells.assign(cell_graph.begin(), cell_graph.end());
    for(int i = 0; i < cells.size(); i++)
    {
        cells[i].isCleaned = true;
    }

    Point2D curr_exit, next_entrance;
    int curr_corner_indicator, next_corner_indicator;

    next_entrance = FindNextEntrance(start, cells[cell_path[1]], next_corner_indicator, robot_radius);
    curr_exit = FindNextEntrance(next_entrance, cells[cell_path[0]], curr_corner_indicator, robot_radius);
    sub_path = WalkingInsideCell(cells[cell_path[0]], start, curr_exit, robot_radius);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    sub_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[0]], cells[cell_path[1]], robot_radius);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    curr_corner_indicator = next_corner_indicator;


    for(int i = 1; i < cell_path.size()-1; i++)
    {
        sub_path = GetBoustrophedonPath(cell_graph, cells[cell_path[i]], curr_corner_indicator, robot_radius);
        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_exit = overall_path.back();
        next_entrance = FindNextEntrance(curr_exit, cells[cell_path[i+1]], next_corner_indicator, robot_radius);

        sub_path = FindLinkingPath(curr_exit, next_entrance, next_corner_indicator, cells[cell_path[i]], cells[cell_path[i+1]], robot_radius);
        overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
        sub_path.clear();

        curr_corner_indicator = next_corner_indicator;
    }

    next_entrance = overall_path.back();
    sub_path = WalkingInsideCell(cells[cell_path.back()], next_entrance, end, robot_radius);
    overall_path.insert(overall_path.end(), sub_path.begin(), sub_path.end());
    sub_path.clear();

    return overall_path;
}

std::vector<CellNode> GenerateCells(cv::Mat map, PolygonList obstacles, int robot_radius)
{
    std::vector<Event> event_list = EventListGenerator(map, obstacles, robot_radius);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    InitializeCellDecomposition(map, cell_graph, cell_index_slice, Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition(map, cell_graph, cell_index_slice, original_cell_index_slice, slice_list);
    FinishCellDecomposition(map, cell_graph, Point2D(slice_list.back().back().x, slice_list.back().back().y));

    return cell_graph;
}

CellNode ContourToCell(cv::Mat map, Polygon contour, int robot_radius)
{
    std::vector<Event> event_list = InitializeEventList(contour, -1);
    EventTypeAllocator2(map, event_list, robot_radius);
    std::sort(event_list.begin(), event_list.end());

    CellNode cell;
    for(int i = 0; i < event_list.size(); i++)
    {
        if(event_list[i].event_type == FLOOR)
        {
            cell.ceiling.emplace_back(Point2D(event_list[i].x, event_list[i].y));
        }
        if(event_list[i].event_type == CEILING)
        {
            cell.floor.emplace_back(Point2D(event_list[i].x, event_list[i].y));
        }
    }
    return cell;
}

std::vector<CellNode> GenerateCells2(const cv::Mat& map, Polygon map_border, PolygonList obstacles, int robot_radius)
{
    CellNode outermost_cell = ContourToCell(map, map_border, robot_radius);

    std::vector<Event> event_list = EventListGenerator(map, obstacles, robot_radius);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    InitializeCellDecomposition2(cell_graph, cell_index_slice, Point2D(slice_list.front().front().x, slice_list.front().front().y), outermost_cell);
    ExecuteCellDecomposition2(cell_graph, cell_index_slice, original_cell_index_slice, slice_list, outermost_cell);
    FinishCellDecomposition2(cell_graph, Point2D(slice_list.back().back().x, slice_list.back().back().y), outermost_cell);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        std::sort(cell_graph[i].ceiling.begin(), cell_graph[i].ceiling.end());
        std::sort(cell_graph[i].floor.begin(), cell_graph[i].floor.end());
    }

    return cell_graph;
}
std::vector<CellNode> GenerateCells2(const cv::Mat& map, CellNode outermost_cell, PolygonList obstacles, int robot_radius)
{
    std::vector<Event> event_list = EventListGenerator(map, obstacles, robot_radius);
    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);

    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice;
    std::vector<int> original_cell_index_slice;
    InitializeCellDecomposition2(cell_graph, cell_index_slice, Point2D(slice_list.front().front().x, slice_list.front().front().y), outermost_cell);
    ExecuteCellDecomposition2(cell_graph, cell_index_slice, original_cell_index_slice, slice_list, outermost_cell);
    FinishCellDecomposition2(cell_graph, Point2D(slice_list.back().back().x, slice_list.back().back().y), outermost_cell);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        std::sort(cell_graph[i].ceiling.begin(), cell_graph[i].ceiling.end());
        std::sort(cell_graph[i].floor.begin(), cell_graph[i].floor.end());
    }

    return cell_graph;
}

std::deque<std::deque<Point2D>> GlobalPathPlanning(cv::Mat map, std::vector<CellNode>& cell_graph, Point2D start_point, int robot_radius, bool visualize_cells=true, bool visualize_path=true, int color_repeats=10)
{
    std::deque<std::deque<Point2D>> global_path;
    std::deque<Point2D> local_path;

    int start_cell_index = DetermineCellIndex(cell_graph, start_point);

    std::deque<Point2D> init_path = PathIninitialization(start_point, cell_graph[start_cell_index], robot_radius);
    local_path.assign(init_path.begin(), init_path.end());

    std::deque<CellNode> cell_path = GetVisittingPath(cell_graph, start_cell_index);

    if(visualize_cells||visualize_path)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::imshow("map", map);
    }

    if(visualize_cells)
    {
        for(int i = 0; i < cell_graph.size(); i++)
        {
            std::cout<<"cell "<<i<<" 's ceiling points number:" << cell_graph[i].ceiling.size()<<std::endl;
            std::cout<<"cell "<<i<<" 's floor points number:" << cell_graph[i].floor.size()<<std::endl;
        }
        std::cout<<"cell graph has "<<cell_graph.size()<<" cells."<<std::endl;
        for(int i = 0; i < cell_graph.size(); i++)
        {
            for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
            {
                std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
            }
        }
        for(int i = 0; i < cell_graph.size(); i++)
        {
            DrawCells(map, cell_graph[i]);
            cv::imshow("map", map);
            cv::waitKey(500);
        }
    }

    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, color_repeats);

    if(visualize_path)
    {
        cv::circle(map, cv::Point(start_point.x, start_point.y), 1, cv::Scalar(0, 0, 255), -1);
        for(int i = 0; i < init_path.size(); i++)
        {
            map.at<cv::Vec3b>(init_path[i].y, init_path[i].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
            UpdateColorMap(JetColorMap);
            cv::imshow("map", map);
            cv::waitKey(1);
        }
    }

    std::deque<Point2D> inner_path;
    std::deque<Point2D> link_path;
    Point2D curr_exit;
    Point2D next_entrance;

    std::deque<int> return_cell_path;
    std::deque<Point2D> return_path;

    int corner_indicator = TOPLEFT;

    for(int i = 0; i < cell_path.size(); i++)
    {
        inner_path = GetBoustrophedonPath(cell_graph, cell_path[i], corner_indicator, robot_radius);
        local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
        global_path.emplace_back(local_path);
        local_path.clear();
        if(visualize_path)
        {
            for(int j = 0; j < inner_path.size(); j++)
            {
                map.at<cv::Vec3b>(inner_path[j].y, inner_path[j].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
                UpdateColorMap(JetColorMap);
                cv::imshow("map", map);
                cv::waitKey(1);
            }
        }

        cell_graph[cell_path[i].cellIndex].isCleaned = true;

        if(i < (cell_path.size()-1))
        {
            curr_exit = inner_path.back();
            next_entrance = FindNextEntrance(curr_exit, cell_path[i+1], corner_indicator, robot_radius);
            link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, cell_path[i], cell_path[i+1], robot_radius);
            local_path.insert(local_path.end(), link_path.begin(), link_path.end());
            if(visualize_path)
            {
                for(int k = 0; k < link_path.size(); k++)
                {
                    map.at<cv::Vec3b>(link_path[k].y, link_path[k].x)=cv::Vec3b(0,0,255);
                    cv::imshow("map", map);
                    cv::waitKey(10);
                }
            }
        }
    }

    if(visualize_cells||visualize_path)
    {
        cv::waitKey(1000);
    }

    return global_path;
}

// 动态规划时，若是有新的障碍物产生，要先用更新后的障碍物队列来重新更新cell_graph再规划
std::deque<Point2D> ReturningPathPlanning(cv::Mat& map, std::vector<CellNode>& cell_graph, Point2D curr_pos, Point2D original_pos, int robot_radius, bool visualize_path=true)
{

    std::deque<int> return_cell_path = FindShortestPath(cell_graph, curr_pos, original_pos);
    std::deque<Point2D> returning_path = WalkingCrossCells(cell_graph, return_cell_path, curr_pos, original_pos, robot_radius);

    if(visualize_path)
    {
        for(int i = 0; i < returning_path.size(); i++)
        {
            map.at<cv::Vec3b>(returning_path[i].y, returning_path[i].x)=cv::Vec3b(255, 255, 255);
            cv::imshow("map", map);
            cv::waitKey(10);
        }
    }

    return returning_path;
}

void InitializeMap(cv::Mat& map)
{
    // TODO
    /**
    读取图片
    二值化
    形态学操作消除空洞和小团块
    最后转化为8UC3格式
    **/
}

PolygonList GetObstacles(cv::Mat original_map, int safe_dist=0) // original_map's type should be 8UC3
{
    // TODO
    /**
     如果最上层的外轮廓是图像边缘，需要将其剔除，提取下一层的所有外轮廓
    **/

    cv::Mat map = original_map.clone();
    map.convertTo(map, CV_8UC1);
    cv::cvtColor(map, map, cv::COLOR_BGR2GRAY);
    int binary_thresh = 200;
    cv::threshold(map, map, binary_thresh, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> original_obstacles;
    std::vector<cv::Vec4i> original_obstacles_hierarcy;
    cv::findContours(map, original_obstacles, original_obstacles_hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  //cv::RETR_EXTERNAL只提取最上层的所有外轮廓


    map = original_map.clone();

    if(safe_dist != 0)
    {
        for(int i = 0; i < original_obstacles.size(); i++)
        {
            for(int j = 0; j < original_obstacles[i].size(); j++)
            {
                cv::circle(map, original_obstacles[i][j], safe_dist, cv::Scalar(255, 255, 255), -1);
            }
        }
    }


    map.convertTo(map, CV_8UC1);
    cv::cvtColor(map, map, cv::COLOR_BGR2GRAY);
    cv::threshold(map, map, binary_thresh, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> dilated_contours;
    std::vector<cv::Vec4i> dilated_contours_hierarcy;
    cv::findContours(map, dilated_contours, dilated_contours_hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);


    std::vector<std::vector<cv::Point>> dilated_obstacles(dilated_contours.size());

    for(int i = 0; i < dilated_contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(dilated_contours[i]), dilated_obstacles[i], 1, true);
    }

    PolygonList obstacles;
    Polygon obstacle;

    for(int i = 0; i < dilated_obstacles.size(); i++)
    {
        for(int j = 0; j < dilated_obstacles[i].size(); j++)
        {
            obstacle.emplace_back(Point2D(dilated_obstacles[i][j].x, dilated_obstacles[i][j].y));
        }
        obstacles.emplace_back(obstacle);
        obstacle.clear();
    }

    return obstacles;
}

Polygon GetMapBorder(cv::Mat map)
{
    // TODO
    /**
    找最上层的内轮廓，且作为该内轮廓的parent轮廓的外轮廓不是图像的边缘
    找到就返回，找不到就使用图像的边缘
    **/
    Polygon map_border;

    return map_border;
}


PolygonList ConstructObstacles(cv::Mat map, std::vector<std::vector<cv::Point>> obstacle_contours)
{
    PolygonList obstacles;

    std::vector<cv::Point> obstacle_contour;
    Polygon obstacle;

    for(int i = 0; i < obstacle_contours.size(); i++)
    {
        obstacle_contour = obstacle_contours[i];

        for(int j = 0; j < obstacle_contour.size()-1; j++)
        {
            cv::LineIterator line(map, obstacle_contour[j], obstacle_contour[j+1]);
            for(int k = 0; k < line.count-1; k++)
            {
                obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
                line++;
            }
        }
        cv::LineIterator line(map, obstacle_contour[obstacle_contour.size()-1], obstacle_contour[0]);
        for(int j = 0; j < line.count-1; j++)
        {
            obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }

        obstacles.emplace_back(obstacle);
    }

    return obstacles;

}

Polygon ConstructDefaultMapBorder(cv::Mat map)
{
    std::vector<cv::Point> default_map_border_contour = {cv::Point(0, 0), cv::Point(0, map.rows-1), cv::Point(map.cols-1, map.rows-1), cv::Point(map.cols-1, 0)};
    std::vector<std::vector<cv::Point>>default_map_border_contours = {default_map_border_contour};

    Polygon default_map_border = ConstructObstacles(map, default_map_border_contours).front();

    return default_map_border;
}



// dynamic complete coverage path planning
// 没有最新的全局地图，只有历史全局地图和当前的碰撞信号
// 目前只适用cell中出现孤立障碍物的情况

const int UP = 0, UPRIGHT = 1, RIGHT = 2, DOWNRIGHT = 3, DOWN = 4, DOWNLEFT = 5, LEFT = 6, UPLEFT = 7, CENTER = 8;
std::vector<int> map_directions = {UP, UPRIGHT, RIGHT, DOWNRIGHT, DOWN, DOWNLEFT, LEFT, UPLEFT};

int GetFrontDirection(Point2D curr_pos, Point2D next_pos)
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
    if(front_direction + 4 > map_directions.size()-1)
    {
        int index_offset = front_direction + 4 - (map_directions.size()-1);
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
    if(front_direction + 2 > map_directions.size()-1)
    {
        int index_offset = front_direction + 2 - (map_directions.size()-1);
        return map_directions[index_offset];
    }
    else
    {
        return map_directions[front_direction+2];
    }
}

// 模拟机器人尝试旋转
// 都是顺时针排列
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


    if(front_direction + 1 > map_directions.size()-1)
    {
        int index_offset = front_direction + 1 - (map_directions.size()-1);
        front_directions.emplace_back(map_directions[index_offset]);
    }
    else
    {
        front_directions.emplace_back(map_directions[front_direction+1]);
    }

    if(front_direction + 2 > map_directions.size()-1)
    {
        int index_offset = front_direction + 2 - (map_directions.size()-1);
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
        if(first_direction + i > map_directions.size()-1)
        {
            int index_offset = first_direction + i - (map_directions.size()-1);
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
        if(front_direction + i > map_directions.size()-1)
        {
            int index_offset = front_direction + i - (map_directions.size()-1);
            right_directions.emplace_back(map_directions[index_offset]);
        }
        else
        {
            right_directions.emplace_back(map_directions[front_direction + i]);
        }
    }

    return right_directions;
}

Point2D GetNextPosition(Point2D curr_pos, int direction, int steps)
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
    }
}

// 模拟碰撞传感器信号
bool CollisionOccurs(cv::Mat map, Point2D curr_pos, int detect_direction, int robot_radius)
{
    int obstacle_dist = INT_MAX;
    Point2D ray_pos;

    for(int i = 1; i <= (robot_radius+1); i++)
    {
        ray_pos = GetNextPosition(curr_pos, detect_direction, i);
        if(map.at<cv::Vec3b>(ray_pos.y, ray_pos.x) == cv::Vec3b(255,255,255))
        {
            obstacle_dist = i;
            break;
        }
    }

    return (obstacle_dist == (robot_radius+1));
}

// TODO：解决障碍物横跨cell的情况，即边界情况
Polygon GetNewObstacle(const cv::Mat& map, Point2D origin, int front_direction, std::deque<Point2D>& contouring_path, int robot_radius)
{
    Point2D curr_pos = origin;
    Point2D last_curr_pos;
    Point2D next_pos;
    std::vector<int> direction_candidates;

    Point2D obstacle_point;
    Polygon new_obstacle;

    int left_direction = GetLeftDirection(front_direction);
    int right_direction = GetRightDirection(front_direction);
    int back_direction = GetBackDirection(front_direction);

    obstacle_point = GetNextPosition(origin, front_direction, robot_radius+1);
    new_obstacle.emplace_back(obstacle_point);

    direction_candidates = GetRightDirectionCandidates(front_direction);
    bool turning = false;
    last_curr_pos = curr_pos;
    while(!turning)
    {
        for (int i = 0; i < direction_candidates.size(); i++) {
            next_pos = GetNextPosition(curr_pos, direction_candidates[i], 1);
            if (CollisionOccurs(map, next_pos, front_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, front_direction, robot_radius+1);
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
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, right_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, front_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }


    direction_candidates = GetFrontDirectionCandidates(front_direction);
    turning = false;
    last_curr_pos = curr_pos;
    while(!turning)
    {
        for (int i = 0; i < direction_candidates.size(); i++) {
            next_pos = GetNextPosition(curr_pos, direction_candidates[i], 1);
            if (CollisionOccurs(map, next_pos, left_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, left_direction, robot_radius+1);
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
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, front_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, left_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }


    direction_candidates = GetLeftDirectionCandidates(front_direction);
    turning = false;
    last_curr_pos = curr_pos;
    while(!turning)
    {
        for (int i = 0; i < direction_candidates.size(); i++) {
            next_pos = GetNextPosition(curr_pos, direction_candidates[i], 1);
            if (CollisionOccurs(map, next_pos, back_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, back_direction, robot_radius+1);
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
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, left_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, back_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }


    direction_candidates = GetBackDirectionCandidates(front_direction);
    turning = false;
    last_curr_pos = curr_pos;
    while(!turning)
    {
        for (int i = 0; i < direction_candidates.size(); i++) {
            next_pos = GetNextPosition(curr_pos, direction_candidates[i], 1);
            if (CollisionOccurs(map, next_pos, right_direction, robot_radius))
            {
                contouring_path.emplace_back(next_pos);
                curr_pos = next_pos;
                obstacle_point = GetNextPosition(next_pos, right_direction, robot_radius+1);
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
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, back_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }
    for(int i = 1; i <= (robot_radius+1); i++)
    {
        next_pos = GetNextPosition(curr_pos, right_direction, 1);
        contouring_path.emplace_back(next_pos);
        curr_pos = next_pos;
    }

    new_obstacle.pop_back();

    return new_obstacle;
}

int GetCleaningDirection(CellNode cell, Point2D exit, int robot_radius)
{
    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell, robot_radius);

    if(exit.x == corner_points[TOPLEFT].x && exit.y == corner_points[TOPLEFT].y)
    {
        return LEFT;
    }
    if(exit.x == corner_points[BOTTOMLEFT].x && exit.y == corner_points[BOTTOMLEFT].y)
    {
        return LEFT;
    }
    if(exit.x == corner_points[TOPRIGHT].x && exit.y == corner_points[TOPRIGHT].y)
    {
        return RIGHT;
    }
    if(exit.x == corner_points[BOTTOMRIGHT].x && exit.y == corner_points[BOTTOMRIGHT].y)
    {
        return RIGHT;
    }
}

// 清扫方向只分向左和向右
// 该cell未清扫过才重新规划
std::deque<std::deque<Point2D>> LocalReplanning(const cv::Mat& map, CellNode outer_cell, PolygonList obstacles, Point2D curr_pos, std::vector<CellNode>& curr_cell_graph, int cleaning_direction, int robot_radius)
{
    //TODO: 边界判断
    int start_x;
    if(cleaning_direction == LEFT)
    {
        if(curr_pos.x + 3 * (robot_radius + 1) <= outer_cell.ceiling.back().x)
        {
            start_x =  curr_pos.x + 3 * (robot_radius + 1);
        }
        else
        {
            start_x = outer_cell.ceiling.back().x;
        }
    }
    if(cleaning_direction == RIGHT)
    {
        if(curr_pos.x - 3 * (robot_radius + 1) >= outer_cell.ceiling.front().x)
        {
            start_x = curr_pos.x - 3 * (robot_radius + 1);
        }
        else
        {
            start_x = outer_cell.ceiling.front().x;
        }
    }
    int outer_cell_index_offset = start_x - outer_cell.ceiling.front().x;

    CellNode inner_cell;
    for(int i = outer_cell_index_offset; i < outer_cell.ceiling.size(); i++)
    {
        inner_cell.ceiling.emplace_back(outer_cell.ceiling[i]);
        inner_cell.floor.emplace_back(outer_cell.floor[i]);
    }

    curr_cell_graph = GenerateCells2(map, inner_cell, obstacles, robot_radius);
    std::deque<std::deque<Point2D>> replanning_path = GlobalPathPlanning(map, curr_cell_graph, curr_pos, robot_radius, true, true);

    return replanning_path;
}

// 每一段都是包含前面的linking_path和后面的inner_path
std::deque<Point2D> DynamicPathPlanning(cv::Mat& map, std::vector<CellNode> global_cell_graph, std::deque<std::deque<Point2D>> global_path, int robot_radius)
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

    int curr_cell_index;
    CellNode curr_cell;

    std::vector<CellNode> overall_cell_graph;
    std::vector<CellNode> curr_cell_graph;

    PolygonList curr_obstacles;

    std::vector<std::deque<std::deque<Point2D>>> unvisited_paths = {global_path};
    std::vector<std::vector<CellNode>> cell_graph_list = {global_cell_graph};
    std::vector<Point2D> exit_list = {global_path.back().back()};

    //
    cv::Mat vismap = map.clone();
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", vismap);
    //



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
                //
                vismap.at<cv::Vec3b>(curr_pos.y, curr_pos.x)=cv::Vec3b(0, 0, 255);
                cv::imshow("map", vismap);
                cv::waitKey(1);
                //

                front_direction = GetFrontDirection(curr_pos, next_pos);
                if(CollisionOccurs(map, curr_pos, front_direction, robot_radius))
                {
                    new_obstacle = GetNewObstacle(map, curr_pos, front_direction, contouring_path, robot_radius);
                    dynamic_path.insert(dynamic_path.end(), contouring_path.begin(), contouring_path.end());
                    //
                    for(int i = 0; i < contouring_path.size(); i++)
                    {
                        vismap.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(0, 0, 255);
                        cv::imshow("map", vismap);
                        cv::waitKey(1);
                    }
                    //
                    contouring_path.clear();

                    curr_cell_index = DetermineCellIndex(curr_cell_graph, curr_pos);
                    curr_cell = curr_cell_graph[curr_cell_index];
                    curr_obstacles={new_obstacle};
                    curr_exit = curr_sub_path.back();
                    cleaning_direction = GetCleaningDirection(curr_cell, curr_exit, robot_radius);

                    replanning_path = LocalReplanning(map, curr_cell, curr_obstacles, dynamic_path.back(), curr_cell_graph, cleaning_direction, robot_radius); // 此处会更新curr_cell_graph
//                    if(!curr_cell.isCleaned)
//                    {
//                        replanning_path = LocalReplanning(map, curr_cell, curr_obstacles, curr_pos, curr_cell_graph, cleaning_direction, robot_radius); // 此处会更新curr_cell_graph
//                    }
//                    else
//                    {
//                        curr_cell_graph = GenerateCells2(map, curr_cell, curr_obstacles, robot_radius);
//                        replanning_path = {ReturningPathPlanning(map, curr_cell_graph, dynamic_path.back(), curr_exit, robot_radius, false)};
//                    }

                    remaining_curr_path.assign(curr_path.begin()+i+1, curr_path.end());

                    goto UPDATING_REMAINING_PATHS;
                }
            }
        }

        if(dynamic_path.back().x != exit_list.back().x && dynamic_path.back().y != exit_list.back().y)
        {
            linking_path = ReturningPathPlanning(map, cell_graph_list.back(), dynamic_path.back(), exit_list.back(), robot_radius, true);
            dynamic_path.insert(dynamic_path.end(), linking_path.begin(), linking_path.end());
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

    return dynamic_path;
}





void StaticPathPlanningTest()
{
    cv::Mat map;
    map = cv::Mat::zeros(500, 500, CV_8UC3);
//    map = cv::Mat::zeros(600, 600, CV_8UC3);

// old data
//    Polygon polygon1, polygon2;
//    cv::LineIterator line1(map, cv::Point(200,300), cv::Point(300,200));
//    cv::LineIterator line2(map, cv::Point(300,200), cv::Point(200,100));
//    cv::LineIterator line3(map, cv::Point(200,100), cv::Point(100,200));
//    cv::LineIterator line4(map, cv::Point(100,200), cv::Point(200,300));
//
//    for(int i = 0; i < line1.count-1; i++)
//    {
//        polygon1.emplace_back(Point2D(line1.pos().x, line1.pos().y));
//        line1++;
//    }
//    for(int i = 0; i < line2.count-1; i++)
//    {
//        polygon1.emplace_back(Point2D(line2.pos().x, line2.pos().y));
//        line2++;
//    }
//    for(int i = 0; i < line3.count-1; i++)
//    {
//        polygon1.emplace_back(Point2D(line3.pos().x, line3.pos().y));
//        line3++;
//    }
//    for(int i = 0; i < line4.count-1; i++)
//    {
//        polygon1.emplace_back(Point2D(line4.pos().x, line4.pos().y));
//        line4++;
//    }
//
//    cv::LineIterator line5(map, cv::Point(300,350), cv::Point(350,300));
//    cv::LineIterator line6(map, cv::Point(350,300), cv::Point(300,250));
//    cv::LineIterator line7(map, cv::Point(300,250), cv::Point(250,300));
//    cv::LineIterator line8(map, cv::Point(250,300), cv::Point(300,350));
//    for(int i = 0; i < line5.count-1; i++)
//    {
//        polygon2.emplace_back(Point2D(line5.pos().x, line5.pos().y));
//        line5++;
//    }
//    for(int i = 0; i < line6.count-1; i++)
//    {
//        polygon2.emplace_back(Point2D(line6.pos().x, line6.pos().y));
//        line6++;
//    }
//    for(int i = 0; i < line7.count-1; i++)
//    {
//        polygon2.emplace_back(Point2D(line7.pos().x, line7.pos().y));
//        line7++;
//    }
//    for(int i = 0; i < line8.count-1; i++)
//    {
//        polygon2.emplace_back(Point2D(line8.pos().x, line8.pos().y));
//        line8++;
//    }
//
//    PolygonList polygons = {polygon1, polygon2};
//
//    std::vector<cv::Point> contour1 = {cv::Point(200,300), cv::Point(300,200), cv::Point(200,100), cv::Point(100,200)};
//    std::vector<cv::Point> contour2 = {cv::Point(300,350), cv::Point(350,300), cv::Point(300,250), cv::Point(250,300)};
//    std::vector<std::vector<cv::Point>> contours = {contour1, contour2};



// new data
    Polygon polygon;
    cv::Point p1(125,125),
            p2(125,175),
            p3(225,175),
            p4(225,225),
            p5(175,250),
            p6(225,300),
            p7(125,325),
            p8(125,375),
            p9(375,375),
            p10(375,325),
            p11(275,325),
            p12(275,275),
            p13(325,250),
            p14(275,200),
            p15(375,175),
            p16(375,125);

//    cv::Point p1(100,100),
//            p2(100,500),
//            p3(150,500),
//            p4(150,150),
//            p5(450,150),
//            p6(450,300),
//            p7(300,300),
//            p8(300,250),
//            p9(350,250),
//            p10(350,200),
//            p11(250,200),
//            p12(250,350),
//            p13(500,350),
//            p14(500,100);


    std::vector<cv::Point> contour = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16};
//    std::vector<cv::Point> contour = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14};

    std::vector<std::vector<cv::Point>> contours = {contour};
    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));







    // code for extracting obstacle's contours


//    cv::namedWindow("original", cv::WINDOW_NORMAL);
//    cv::imshow("original", map);
//    cv::waitKey(0);
//
//    std::vector<cv::Point> contour_;
//    std::vector<std::vector<cv::Point>> contours_;
//
//    for(int i = 0; i < contour.size()-1; i++)
//    {
//        cv::LineIterator line(map, contour[i], contour[i+1]);
//        for(int j = 0; j < line.count-1; j++)
//        {
//            contour_.emplace_back(cv::Point(line.pos().x, line.pos().y));
//            line++;
//        }
//    }
//    cv::LineIterator line(map, contour[contour.size()-1], contour[0]);
//    for(int i = 0; i < line.count-1; i++)
//    {
//        contour_.emplace_back(cv::Point(line.pos().x, line.pos().y));
//        line++;
//    }
//
//    contours_ = {contour_};
//
//    int robot_radius = 10;
//    for(int i = 0; i < contours_.size(); i++)
//    {
//        for(int j = 0; j < contours_[i].size(); j++)
//        {
//            cv::circle(map, contours_[i][j], robot_radius, cv::Scalar(255, 255, 255), -1);
//        }
//    }
//
//    cv::namedWindow("dilated", cv::WINDOW_NORMAL);
//    cv::imshow("dilated", map);
//    cv::waitKey(0);
//
//
//
//
//    std::vector<std::vector<cv::Point>> dilated_contours;
//    std::vector<cv::Vec4i> hierarcy;
//
//
//    cv::Mat map_;
//    map.convertTo(map_, CV_8UC1);
//    cv::cvtColor(map_, map_, cv::COLOR_BGR2GRAY);
//    cv::threshold(map_, map_, 200, 255, cv::THRESH_BINARY);
//
//    cv::findContours(map_, dilated_contours, hierarcy, 0, cv::CHAIN_APPROX_NONE);
//    std::vector<std::vector<cv::Point>> polygon_contours(dilated_contours.size());
//
//    for(int i = 0; i < dilated_contours.size(); i++)
//    {
//        cv::approxPolyDP(cv::Mat(dilated_contours[i]), polygon_contours[i], 1, true);
//    }
//
//    for(int i = 0; i < polygon_contours.size(); i++)
//    {
//        cv::drawContours(map, polygon_contours, i, cv::Scalar(0, 0, 255));
//    }
//
//    cv::namedWindow("polygoncontour", cv::WINDOW_NORMAL);
//    cv::imshow("polygoncontour", map);
//    cv::waitKey(0);








    for(int i = 0; i < contour.size()-1; i++)
    {
        cv::LineIterator line(map, contour[i], contour[i+1]);
        for(int j = 0; j < line.count-1; j++)
        {
            polygon.emplace_back(Point2D(line.pos().x, line.pos().y));
            line++;
        }
    }
    cv::LineIterator line(map, contour[contour.size()-1], contour[0]);
    for(int i = 0; i < line.count-1; i++)
    {
        polygon.emplace_back(Point2D(line.pos().x, line.pos().y));
        line++;
    }

    PolygonList polygons = {polygon};

    int robot_radius = 5;
    std::vector<Event> event_list = EventListGenerator(map, polygons, robot_radius);

    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
    std::vector<CellNode> cell_graph;
    std::vector<int> cell_index_slice; // 按y从小到大排列
    std::vector<int> original_cell_index_slice;

    InitializeCellDecomposition(map, cell_graph, cell_index_slice, Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition(map, cell_graph, cell_index_slice, original_cell_index_slice, slice_list);
    FinishCellDecomposition(map, cell_graph, Point2D(slice_list.back().back().x, slice_list.back().back().y));

    Point2D start_point = Point2D(10, 10);
    Point2D end_point;
    int start_cell_index = 0;

    std::deque<Point2D> first_steps = PathIninitialization(start_point, cell_graph[start_cell_index], robot_radius);

    std::deque<CellNode> path = GetVisittingPath(cell_graph, start_cell_index);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        std::cout<<"cell "<<i<<" 's ceiling points number:" << cell_graph[i].ceiling.size()<<std::endl;
        std::cout<<"cell "<<i<<" 's floor points number:" << cell_graph[i].floor.size()<<std::endl;
    }

    std::cout<<cell_graph.size()<<std::endl;


    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
        {
            std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
        }
    }

    int repeat_times = 30;
    std::deque<cv::Scalar> JetColorMap;
    InitializeColorMap(JetColorMap, repeat_times);

    cv::circle(map, cv::Point(start_point.x, start_point.y), 3, cv::Scalar(0, 0, 255), -1);

    cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
    cv::imshow("trajectory", map);
//    cv::waitKey(0);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        DrawCells(map, cell_graph[i]);
        cv::imshow("trajectory", map);
        cv::waitKey(500);
    }


    for(int i = 0; i < first_steps.size(); i++)
    {
        map.at<cv::Vec3b>(first_steps[i].y, first_steps[i].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
        UpdateColorMap(JetColorMap);
        cv::imshow("trajectory", map);
        cv::waitKey(1);
    }

    std::deque<Point2D> sub_path;
    int corner_indicator = TOPLEFT;

    for(int i = 0; i < path.size(); i++)
    {
        sub_path = GetBoustrophedonPath(cell_graph, path[i], corner_indicator, robot_radius);
        for(int j = 0; j < sub_path.size(); j++)
        {
            map.at<cv::Vec3b>(sub_path[j].y, sub_path[j].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
            UpdateColorMap(JetColorMap);
            cv::imshow("trajectory", map);
            cv::waitKey(1);
        }

        cell_graph[path[i].cellIndex].isCleaned = true;

        if(i < (path.size()-1))
        {
            Point2D curr_exit = sub_path.back();
            Point2D next_entrance = FindNextEntrance(curr_exit, path[i + 1], corner_indicator, robot_radius);
            std::deque<Point2D> link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, path[i], path[i+1], robot_radius);
            for(int k = 0; k < link_path.size(); k++)
            {
                map.at<cv::Vec3b>(link_path[k].y, link_path[k].x)=cv::Vec3b(0,0,255);
//                map.at<cv::Vec3b>(link_path[k].y, link_path[k].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
//                UpdateColorMap();
                cv::imshow("trajectory", map);
                cv::waitKey(10);
            }
        }
    }

    end_point = sub_path.back();
    std::deque<int> return_cell_path = FindShortestPath(cell_graph, end_point, start_point);
    std::deque<Point2D> return_path = WalkingCrossCells(cell_graph, return_cell_path, end_point, start_point, robot_radius);
    for(int i = 0; i < return_path.size(); i++)
    {
        map.at<cv::Vec3b>(return_path[i].y, return_path[i].x)=cv::Vec3b(255, 255, 255);
//        UpdateColorMap();
        cv::imshow("trajectory", map);
        cv::waitKey(10);
    }

    cv::waitKey(0);



    // code for test of finding correct event types

//    for(int i = 0; i < event_list.size(); i++)
//    {
//        if(event_list[i].event_type == IN)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
//        }
//        if(event_list[i].event_type == IN_TOP)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN_TOP"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
//        }
//        if(event_list[i].event_type == IN_BOTTOM)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN_BOTTOM"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
//        }
//        if(event_list[i].event_type == OUT)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
//        }
//        if(event_list[i].event_type == OUT_TOP)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT_TOP"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
//        }
//        if(event_list[i].event_type == OUT_BOTTOM)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT_BOTTOM"<<std::endl;
//            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
//        }
//        if(event_list[i].event_type == INNER_IN)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN"<<std::endl;
//        }
//        if(event_list[i].event_type == INNER_IN_TOP)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN_TOP"<<std::endl;
//        }
//        if(event_list[i].event_type == INNER_IN_BOTTOM)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN_BOTTOM"<<std::endl;
//        }
//        if(event_list[i].event_type == INNER_OUT)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT"<<std::endl;
//        }
//        if(event_list[i].event_type == INNER_OUT_TOP)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT_TOP"<<std::endl;
//        }
//        if(event_list[i].event_type == INNER_OUT_BOTTOM)
//        {
//            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT_BOTTOM"<<std::endl;
//        }
//        if(event_list[i].event_type == MIDDLE)
//        {
//            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(50, 50 ,50);
//        }
//        if(event_list[i].event_type == CEILING)
//        {
//            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(0, 255 ,255);
//        }
//        if(event_list[i].event_type == FLOOR)
//        {
//            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(255, 0 ,0);
//        }
//    }
//
//    cv::namedWindow("map", cv::WINDOW_NORMAL);
//    cv::imshow("map", map);
//    cv::waitKey(0);
}

void PointTypeTest(cv::Mat& map, Polygon obstacle, int robot_radius)
{
    PolygonList obstacles = {obstacle};

    std::vector<Event> event_list = EventListGenerator(map, obstacles, robot_radius);
    for(int i = 0; i < event_list.size(); i++)
    {
        if(event_list[i].event_type == IN)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
        }
        if(event_list[i].event_type == IN_TOP)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN_TOP"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
        }
        if(event_list[i].event_type == IN_BOTTOM)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", IN_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 0, 255), -1);//bright red
        }
        if(event_list[i].event_type == OUT)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
        }
        if(event_list[i].event_type == OUT_TOP)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT_TOP"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
        }
        if(event_list[i].event_type == OUT_BOTTOM)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", OUT_BOTTOM"<<std::endl;
            cv::circle(map, cv::Point(event_list[i].x, event_list[i].y), 2, cv::Scalar(0, 255, 0), -1);//bright green
        }
        if(event_list[i].event_type == INNER_IN)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN"<<std::endl;
        }
        if(event_list[i].event_type == INNER_IN_TOP)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN_TOP"<<std::endl;
        }
        if(event_list[i].event_type == INNER_IN_BOTTOM)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_IN_BOTTOM"<<std::endl;
        }
        if(event_list[i].event_type == INNER_OUT)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT"<<std::endl;
        }
        if(event_list[i].event_type == INNER_OUT_TOP)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT_TOP"<<std::endl;
        }
        if(event_list[i].event_type == INNER_OUT_BOTTOM)
        {
            std::cout<<event_list[i].x<<", "<<event_list[i].y<<", INNER_OUT_BOTTOM"<<std::endl;
        }
        if(event_list[i].event_type == MIDDLE)
        {
            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(50, 50 ,50);
        }
        if(event_list[i].event_type == CEILING)
        {
            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(0, 255 ,255);
        }
        if(event_list[i].event_type == FLOOR)
        {
            map.at<cv::Vec3b>(event_list[i].y, event_list[i].x) = cv::Vec3b(255, 0 ,0);
        }
    }
}

int main() {

//    StaticPathPlanningTest();

    int robot_radius = 5;
    cv::Mat history_map = cv::Mat::zeros(500, 500, CV_8UC3);
    std::vector<cv::Point> original_obstacle_contour1 = {cv::Point(125, 50), cv::Point(50, 125), cv::Point(125, 200), cv::Point(200, 125)};
    std::vector<std::vector<cv::Point>> original_obstacle_contours = {original_obstacle_contour1};
    cv::fillPoly(history_map, original_obstacle_contours, cv::Scalar(255, 255, 255));
    PolygonList original_obstacles = ConstructObstacles(history_map, original_obstacle_contours);
    Polygon original_map_border = ConstructDefaultMapBorder(history_map);
    std::vector<CellNode> original_cell_graph = GenerateCells2(history_map, original_map_border, original_obstacles, robot_radius);
    Point2D start = Point2D(10, 10);
    std::deque<std::deque<Point2D>> original_planning_path = GlobalPathPlanning(history_map, original_cell_graph, start, robot_radius, false, false);




    cv::Mat curr_map = cv::Mat::zeros(500, 500, CV_8UC3);
    std::vector<cv::Point> temp_obstacle_contour1 = {cv::Point(80, 300), cv::Point(80, 400), cv::Point(160, 400), cv::Point(160, 300)}; //cv::Point(120, 350),
    std::vector<cv::Point> temp_obstacle_contour2 = {cv::Point(300, 150), cv::Point(300, 250), cv::Point(400, 220), cv::Point(400, 180)};
    std::vector<std::vector<cv::Point>> curr_obstacle_contours = {original_obstacle_contour1, temp_obstacle_contour1, temp_obstacle_contour2};
    cv::fillPoly(curr_map, curr_obstacle_contours, cv::Scalar(255, 255, 255));
    PolygonList curr_obstacles = ConstructObstacles(curr_map, curr_obstacle_contours);


//    for(int i = 0; i < original_planning_path.size(); i++)
//    {
//        for(int j = 0; j < original_planning_path[i].size(); j++)
//        {
//            curr_map.at<cv::Vec3b>(original_planning_path[i][j].y, original_planning_path[i][j].x)=cv::Vec3b(50, 50, 50);
//        }
//    }

    std::deque<Point2D> dynamic_path = DynamicPathPlanning(curr_map, original_cell_graph, original_planning_path, robot_radius);



//    Point2D collision_point = Point2D(80, 406);
//    std::deque<Point2D> contouring_path;
//    Polygon new_obstacle = GetNewObstacle(curr_map, collision_point, UP, contouring_path, robot_radius);
//    for(int i = 0; i < contouring_path.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(contouring_path[i].y, contouring_path[i].x)=cv::Vec3b(0, 0, 255);
//    }
//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(new_obstacle[i].y, new_obstacle[i].x)=cv::Vec3b(0, 255, 0);
//    }
//
//    for(int i = 0; i < new_obstacle.size(); i++)
//    {
//        std::cout<<"x: "<<new_obstacle[i].x<<", y: "<<new_obstacle[i].y<<std::endl;
//    }





//    int repeat_times = 30;
//    std::deque<cv::Scalar> JetColorMap;
//    InitializeColorMap(JetColorMap, repeat_times);
//    for(int i = 0; i < dynamic_path.size(); i++)
//    {
//        curr_map.at<cv::Vec3b>(dynamic_path[i].y, dynamic_path[i].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
//        UpdateColorMap(JetColorMap);
//        cv::imshow("curr_map", curr_map);
//        cv::waitKey(1);
//    }

    return 0;
}