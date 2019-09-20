#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat map;

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
    int x;
    int y;
};

typedef std::vector<Point2D> Polygon;   // contour points extracted from a blob, sorted by counter clockwise manner
typedef std::vector<Polygon> PolygonList;
typedef std::vector<Point2D> Edge;

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


std::deque<CellNode> path;

std::vector<CellNode> cell_graph;

int unvisited_counter = INT_MAX;

void WalkingThroughGraph(int cell_index)
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
        WalkingThroughGraph(neighbor_idx);
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
            WalkingThroughGraph(cell_graph[cell_index].parentIndex);
        }
    }
}

std::vector<Point2D> GetBoustrophedonPath(CellNode cell, int corner_indicator, int robot_radius=0)
{
    std::vector<Point2D> ceiling = cell.ceiling;
    std::vector<Point2D> floor = cell.floor;

    std::vector<Point2D> path;

    if(cell_graph[cell.cellIndex].isCleaned)
    {
        int c_size = cell.ceiling.size();
        int f_size = cell.floor.size();

        if(corner_indicator == TOPLEFT)
        {
            path.emplace_back(Point2D(ceiling[(robot_radius + 1)].x, ceiling[(robot_radius + 1)].y + (robot_radius + 1)));
        }
        if(corner_indicator == TOPRIGHT)
        {
            path.emplace_back(Point2D(cell.ceiling[c_size-1-(robot_radius + 1)].x, cell.ceiling[c_size-1-(robot_radius + 1)].y + (robot_radius + 1)));
        }
        if(corner_indicator == BOTTOMLEFT)
        {
            path.emplace_back(Point2D(cell.floor[robot_radius + 1].x, cell.floor[robot_radius + 1].y - (robot_radius + 1)));
        }
        if(corner_indicator == BOTTOMRIGHT)
        {
            path.emplace_back(Point2D(cell.floor[f_size-1-(robot_radius + 1)].x, cell.floor[f_size-1-(robot_radius + 1)].y - (robot_radius + 1)));
        }
    }
    else
    {
        if(corner_indicator == TOPLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i=i+robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, floor[i+j].y-(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, ceiling[i+j].y+(robot_radius + 1)));
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

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, floor[i-j].y-(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for (y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, ceiling[i-j].y+(robot_radius + 1)));
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

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i=i+robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, ceiling[i+j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, floor[i+j].y-(robot_radius + 1)));
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

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, ceiling[i-j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, floor[i-j].y-(robot_radius + 1)));
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

// 各个多边形按照其左顶点的x值从小到大的顺序进行排列
// 且对于每个多边形，其最左边和最右边都只有唯一的一个点
std::vector<Event> EventListGenerator(PolygonList polygons)
{
    std::vector<Event> event_list;

    Polygon polygon;
    int leftmost_idx, rightmost_idx;

    for(int i = 0; i < polygons.size(); i++)
    {
        polygon = polygons[i];
        leftmost_idx = std::distance(polygon.begin(),std::min_element(polygon.begin(),polygon.end()));
        rightmost_idx = std::distance(polygon.begin(), std::max_element(polygon.begin(),polygon.end()));

        event_list.emplace_back(Event(i, polygon[leftmost_idx].x, polygon[leftmost_idx].y, IN));
        event_list.emplace_back(Event(i, polygon[rightmost_idx].x, polygon[rightmost_idx].y, OUT));

        // 假设多边形为凸壳 且各个顶点按照逆时针的顺序排列
        if (leftmost_idx < rightmost_idx)
        {
            for(int m = 0; m < polygon.size(); m++)
            {
                if(leftmost_idx < m && m < rightmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[m].x, polygon[m].y, CEILING));
                }
                if(m < leftmost_idx || m >rightmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[m].x, polygon[m].y, FLOOR));
                }
            }
        }
        else if(leftmost_idx > rightmost_idx)
        {
            for(int n = 0; n < polygon.size(); n++)
            {
                if(rightmost_idx < n && n < leftmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[n].x, polygon[n].y, FLOOR));
                }
                if(n < rightmost_idx || n > leftmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[n].x, polygon[n].y, CEILING));
                }
            }
        }
    }
    std::sort(event_list.begin(),event_list.end());
    return event_list;
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


void EventTypeAllocator(std::vector<Event>& event_list)
{
    int half_size = event_list.size()%2==0? event_list.size()/2 : (event_list.size()+1)/2;
    std::vector<Event> header(event_list.begin()+half_size, event_list.end());
    std::vector<Event> tail(event_list.begin(), event_list.begin()+half_size);
    std::vector<Event> event_list_ex;
    event_list_ex.insert(event_list_ex.begin(), header.begin(), header.end());
    event_list_ex.insert(event_list_ex.end(), event_list.begin(), event_list.end());
    event_list_ex.insert(event_list_ex.end(), tail.begin(), tail.end());

    int index_offset;
    std::deque<int> index_list;

    // determine in and out
    for(int i = half_size; i < half_size + event_list.size(); i++)
    {
        if(event_list_ex[i].x < event_list_ex[i-1].x && event_list_ex[i].x < event_list_ex[i+1].x)
        {
            event_list[i-half_size].event_type = IN;
            index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
            }
        }


        if(event_list_ex[i].x > event_list_ex[i-1].x && event_list_ex[i].x > event_list_ex[i+1].x)
        {
            event_list[i-half_size].event_type = OUT;
            index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
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
                index_list.emplace_back(i-half_size);
            }
        }


    }


    // determine inner
    int temp_index;

    for(int i = 1; i <= index_list.size(); i++)
    {
        if(event_list[index_list[1]].event_type == OUT)
        {
            if(
              (event_list[index_list[0]].event_type == IN
               && event_list[index_list[2]].event_type == IN
               && event_list[index_list[0]].y < event_list[index_list[1]].y
               && event_list[index_list[2]].y > event_list[index_list[1]].y)

            ||(event_list[index_list[0]].event_type == IN_BOTTOM
               && event_list[index_list[2]].event_type == IN_TOP)

            ||(event_list[index_list[0]].event_type == IN_BOTTOM
               && event_list[index_list[2]].event_type == IN
               && event_list[index_list[2]].y > event_list[index_list[1]].y)

            ||(event_list[index_list[0]].event_type == IN
               && event_list[index_list[0]].y < event_list[index_list[1]].y
               && event_list[index_list[2]].event_type == IN_TOP)
            )
            {
                event_list[index_list[1]].event_type = INNER_OUT;
            }
            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }

        if(event_list[index_list[1]].event_type == OUT_TOP)
        {
            // TODO: case: index_list length less than 4

            if(
               (  event_list[index_list[0]].event_type == IN
               && (event_list[index_list[2]].event_type == OUT_BOTTOM || event_list[index_list[2]].event_type == INNER_OUT_BOTTOM)
               && event_list[index_list[3]].event_type == IN
               && event_list[index_list[0]].y < event_list[index_list[1]].y
               && event_list[index_list[3]].y > event_list[index_list[1]].y)

               ||(event_list[index_list[0]].event_type == IN_BOTTOM
                  && (event_list[index_list[2]].event_type == OUT_BOTTOM || event_list[index_list[2]].event_type == INNER_OUT_BOTTOM)
                  && event_list[index_list[3]].event_type == IN_TOP)

               ||(event_list[index_list[0]].event_type == IN
                  && (event_list[index_list[2]].event_type == OUT_BOTTOM || event_list[index_list[2]].event_type == INNER_OUT_BOTTOM)
                  && event_list[index_list[3]].event_type == IN_TOP
                  && event_list[index_list[0]].y < event_list[index_list[1]].y
                  )

               ||(event_list[index_list[0]].event_type == IN_BOTTOM
                  && (event_list[index_list[2]].event_type == OUT_BOTTOM || event_list[index_list[2]].event_type == INNER_OUT_BOTTOM)
                  && event_list[index_list[3]].event_type == IN
                  && event_list[index_list[3]].y > event_list[index_list[1]].y)
              )
            {
                event_list[index_list[1]].event_type = INNER_OUT_TOP;
            }

            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }



        if(event_list[index_list[1]].event_type == OUT_BOTTOM)
        {
            if(
               (event_list[index_list[index_list.size()-1]].event_type == IN
               && (event_list[index_list[0]].event_type == OUT_TOP || event_list[index_list[0]].event_type == INNER_OUT_TOP)
               && event_list[index_list[2]].event_type == IN
               && event_list[index_list[index_list.size()-1]].y < event_list[index_list[1]].y
               && event_list[index_list[2]].y > event_list[index_list[1]].y)

               ||(event_list[index_list[index_list.size()-1]].event_type == IN_BOTTOM
                  && (event_list[index_list[0]].event_type == OUT_TOP || event_list[index_list[0]].event_type == INNER_OUT_TOP)
                  && event_list[index_list[2]].event_type == IN_TOP)

               ||(event_list[index_list[index_list.size()-1]].event_type == IN
                  && (event_list[index_list[0]].event_type == OUT_TOP || event_list[index_list[0]].event_type == INNER_OUT_TOP)
                  && event_list[index_list[2]].event_type == IN_TOP
                  && event_list[index_list[index_list.size()-1]].y < event_list[index_list[1]].y)

               ||(event_list[index_list[index_list.size()-1]].event_type == IN_BOTTOM
                  && (event_list[index_list[0]].event_type == OUT_TOP || event_list[index_list[0]].event_type == INNER_OUT_TOP)
                  && event_list[index_list[2]].event_type == IN
                  && event_list[index_list[2]].y > event_list[index_list[1]].y)
               )
            {
                event_list[index_list[1]].event_type = INNER_OUT_BOTTOM;
            }


            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }

        if(event_list[index_list[1]].event_type == IN)
        {
            if(
               (event_list[index_list[0]].event_type == OUT
               && event_list[index_list[2]].event_type == OUT
               && event_list[index_list[0]].y > event_list[index_list[1]].y
               && event_list[index_list[2]].y < event_list[index_list[1]].y)

               ||(event_list[index_list[0]].event_type == OUT_TOP
                  && event_list[index_list[2]].event_type == OUT_BOTTOM)

               ||(event_list[index_list[0]].event_type == OUT
                  && event_list[index_list[2]].event_type == OUT_BOTTOM
                  && event_list[index_list[0]].y > event_list[index_list[1]].y)

               ||(event_list[index_list[0]].event_type == OUT_TOP
                  && event_list[index_list[2]].event_type == OUT
                  && event_list[index_list[2]].y < event_list[index_list[1]].y)
               )
            {
                event_list[index_list[1]].event_type = INNER_IN;
            }

            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }


        if(event_list[index_list[1]].event_type == IN_TOP)
        {
            if(
               (event_list[index_list[index_list.size()-1]].event_type == OUT
               && (event_list[index_list[0]].event_type == IN_BOTTOM || event_list[index_list[0]].event_type == INNER_IN_BOTTOM)
               && event_list[index_list[2]].event_type == OUT
               && event_list[index_list[index_list.size()-1]].y > event_list[index_list[1]].y
               && event_list[index_list[2]].y < event_list[index_list[1]].y)

               ||(event_list[index_list[index_list.size()-1]].event_type == OUT_TOP
                  && (event_list[index_list[0]].event_type == IN_BOTTOM || event_list[index_list[0]].event_type == INNER_IN_BOTTOM)
                  && event_list[index_list[2]].event_type == OUT_BOTTOM)

               ||(event_list[index_list[index_list.size()-1]].event_type == OUT
                  && (event_list[index_list[0]].event_type == IN_BOTTOM || event_list[index_list[0]].event_type == INNER_IN_BOTTOM)
                  && event_list[index_list[2]].event_type == OUT_BOTTOM
                  && event_list[index_list[index_list.size()-1]].y > event_list[index_list[1]].y)

               ||(event_list[index_list[index_list.size()-1]].event_type == OUT_TOP
                  && (event_list[index_list[0]].event_type == IN_BOTTOM || event_list[index_list[0]].event_type == INNER_IN_BOTTOM)
                  && event_list[index_list[2]].event_type == OUT
                  && event_list[index_list[2]].y < event_list[index_list[1]].y)
               )
            {
                event_list[index_list[1]].event_type = INNER_IN_TOP;
            }

            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }

        if(event_list[index_list[1]].event_type == IN_BOTTOM)
        {
            if(
               (event_list[index_list[0]].event_type == OUT
               && (event_list[index_list[2]].event_type == IN_TOP || event_list[index_list[2]].event_type == INNER_IN_TOP)
               && event_list[index_list[3]].event_type == OUT
               && event_list[index_list[0]].y > event_list[index_list[1]].y
               && event_list[index_list[3]].y < event_list[index_list[1]].y)

               ||(event_list[index_list[0]].event_type == OUT_TOP
                  && (event_list[index_list[2]].event_type == IN_TOP || event_list[index_list[2]].event_type == INNER_IN_TOP)
                  && event_list[index_list[3]].event_type == OUT_BOTTOM)

               ||(event_list[index_list[0]].event_type == OUT
                  && (event_list[index_list[2]].event_type == IN_TOP || event_list[index_list[2]].event_type == INNER_IN_TOP)
                  && event_list[index_list[3]].event_type == OUT_BOTTOM
                  && event_list[index_list[0]].y > event_list[index_list[1]].y)

               ||(event_list[index_list[0]].event_type == OUT_TOP
                  && (event_list[index_list[2]].event_type == IN_TOP || event_list[index_list[2]].event_type == INNER_IN_TOP)
                  && event_list[index_list[3]].event_type == OUT
                  && event_list[index_list[3]].y < event_list[index_list[1]].y)
               )
            {
                event_list[index_list[1]].event_type = INNER_IN_BOTTOM;
            }

            temp_index = index_list.front();
            index_list.pop_front();
            index_list.emplace_back(temp_index);
        }
    }


    // determine floor and ceiling
    for(int i = 1; i <= index_list.size(); i++)
    {
        if(
           (event_list[index_list[0]].event_type == OUT
           && event_list[index_list[1]].event_type == IN)

           ||(event_list[index_list[0]].event_type == OUT
              && event_list[index_list[1]].event_type == INNER_IN)

           ||(event_list[index_list[0]].event_type == OUT
              && event_list[index_list[1]].event_type == INNER_IN_BOTTOM)

           ||(event_list[index_list[0]].event_type == OUT_TOP
              && event_list[index_list[1]].event_type == IN)

           ||(event_list[index_list[0]].event_type == OUT_TOP
              && event_list[index_list[1]].event_type == IN_TOP)

           ||(event_list[index_list[0]].event_type == OUT_TOP
              && event_list[index_list[1]].event_type == INNER_IN)

           ||(event_list[index_list[0]].event_type == OUT_TOP
              && event_list[index_list[1]].event_type == INNER_IN_BOTTOM)

           ||(event_list[index_list[0]].event_type == INNER_OUT
              && event_list[index_list[1]].event_type == IN)

           ||(event_list[index_list[0]].event_type == INNER_OUT
              && event_list[index_list[1]].event_type == IN_TOP)

           ||(event_list[index_list[0]].event_type == INNER_OUT_BOTTOM
              && event_list[index_list[1]].event_type == IN)

           ||(event_list[index_list[0]].event_type == INNER_OUT_BOTTOM
              && event_list[index_list[1]].event_type == IN_TOP)

           )
        {
            if(index_list[0] < index_list[1])
            {
                for(int j = index_list[0]+1; j < index_list[1]; j++)
                {
                    event_list[j].event_type = FLOOR;
                }
            }
            else
            {
                for(int j = index_list[0]+1; j < event_list.size(); j++)
                {
                    event_list[j].event_type = FLOOR;
                }
                for(int k = 0; k < index_list[1]; k++)
                {
                    event_list[k].event_type = FLOOR;
                }
            }
        }

        if(
           (event_list[index_list[0]].event_type == IN
           && event_list[index_list[1]].event_type == OUT)

           ||(event_list[index_list[0]].event_type == IN
              && event_list[index_list[1]].event_type == INNER_OUT)

           ||(event_list[index_list[0]].event_type == IN
              && event_list[index_list[1]].event_type == INNER_OUT_TOP)

           ||(event_list[index_list[0]].event_type == IN_BOTTOM
              && event_list[index_list[1]].event_type == OUT)

           ||(event_list[index_list[0]].event_type == IN_BOTTOM
              && event_list[index_list[1]].event_type == OUT_BOTTOM)

           ||(event_list[index_list[0]].event_type == IN_BOTTOM
              && event_list[index_list[1]].event_type == INNER_OUT)

           ||(event_list[index_list[0]].event_type == IN_BOTTOM
              && event_list[index_list[1]].event_type == INNER_OUT_TOP)

           ||(event_list[index_list[0]].event_type == INNER_IN
              && event_list[index_list[1]].event_type == OUT)

           ||(event_list[index_list[0]].event_type == INNER_IN
              && event_list[index_list[1]].event_type == OUT_BOTTOM)

           ||(event_list[index_list[0]].event_type == INNER_IN_TOP
              && event_list[index_list[1]].event_type == OUT)

           ||(event_list[index_list[0]].event_type == INNER_IN_TOP
              && event_list[index_list[1]].event_type == OUT_BOTTOM)
           )
        {
            if(index_list[0] < index_list[1])
            {
                for(int j = index_list[0]+1; j < index_list[1]; j++)
                {
                    event_list[j].event_type = CEILING;
                }
            }
            else
            {
                for(int j = index_list[0]+1; j < event_list.size(); j++)
                {
                    event_list[j].event_type = CEILING;
                }
                for(int k = 0; k < index_list[1]; k++)
                {
                    event_list[k].event_type = CEILING;
                }
            }
        }

        temp_index = index_list.front();
        index_list.pop_front();
        index_list.emplace_back(temp_index);
    }
}

std::vector<Event> EventListGenerator2(PolygonList polygons)
{
    std::vector<Event> event_list;
    std::vector<Event> event_sublist;

    for(int i = 0; i < polygons.size(); i++)
    {
        event_sublist = InitializeEventList(polygons[i], i);
        EventTypeAllocator(event_sublist);
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



void ExecuteOpenOperation(int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false) // in event  add two new node
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(Point2D(in.x, c.y));
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(Point2D(in.x, f.y));

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

void ExecuteCloseOperation(int top_cell_idx, int bottom_cell_idx, Point2D out, Point2D c, Point2D f, bool rewrite = false) // out event  add one new node
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(Point2D(out.x, c.y));
    new_cell.floor.emplace_back(Point2D(out.x, f.y));

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



void ExecuteCeilOperation(int curr_cell_idx, const Point2D& ceil_point) // finish constructing last ceiling edge
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void ExecuteFloorOperation(int curr_cell_idx, const Point2D& floor_point) // finish constructing last floor edge
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}



void ExecuteOpenOperation(int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false) // in event  add two new node
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(Point2D(in_top.x, c.y));
    top_cell.floor.emplace_back(in_top);

    bottom_cell.ceiling.emplace_back(in_bottom);
    bottom_cell.floor.emplace_back(Point2D(in_bottom.x, f.y));


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

void ExecuteCloseOperation(int top_cell_idx, int bottom_cell_idx, Point2D out_top, Point2D out_bottom, Point2D c, Point2D f, bool rewrite = false) // out event  add one new node
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(Point2D(out_top.x, c.y));
    new_cell.floor.emplace_back(Point2D(out_bottom.x, f.y));

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




void ExecuteInnerOpenOperation(Point2D inner_in)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in);
    new_cell.floor.emplace_back(inner_in);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void ExecuteInnerOpenOperation(Point2D inner_in_top, Point2D inner_in_bottom)
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(inner_in_top);
    new_cell.floor.emplace_back(inner_in_bottom);

    int new_cell_index = cell_graph.size();

    new_cell.cellIndex = new_cell_index;
    cell_graph.emplace_back(new_cell);
}

void ExecuteInnerCloseOperation(int curr_cell_idx, Point2D inner_out)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
}

void ExecuteInnerCloseOperation(int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom)
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
    cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
}






std::vector<int> cell_index_slice; // 按y从小到大排列
std::vector<int> original_cell_index_slice;

void InitializeCellDecomposition(Point2D first_in_pos)
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
}

void FinishCellDecomposition(Point2D last_out_pos)
{
    int last_cell_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点
    for(int i = last_out_pos.x + 1; i <= map.cols-1; i++)
    {
        cell_graph[last_cell_idx].ceiling.emplace_back(Point2D(i, 0));
        cell_graph[last_cell_idx].floor.emplace_back(Point2D(i, map.rows-1));
    }

    unvisited_counter = cell_graph.size();
}


void DrawCells(const CellNode& cell)
{
    for(int i = 0; i < cell.ceiling.size(); i++)
    {
        map.at<cv::Vec3b>(cell.ceiling[i].y, cell.ceiling[i].x) = cv::Vec3b(0, 0, 255); // 96 96 96
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        map.at<cv::Vec3b>(cell.floor[i].y, cell.floor[i].x) = cv::Vec3b(0, 0, 255);
    }

//    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), cv::Scalar(0, 0, 255));
//    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), cv::Scalar(0, 0, 255));
}


std::deque<Event> SortSliceEvent(const std::deque<Event>& slice)
{
    std::deque<Event> sorted_slice(slice);
    std::deque<Event> sub_slice;

    for(int i = 0; i < sorted_slice.size(); i++)
    {
        sorted_slice[i].original_index_in_slice = i;
    }

    for(int i = 0; i < sorted_slice.size(); i++)
    {
        if(sorted_slice[i].event_type == IN)
        {
            sub_slice.emplace_back(sorted_slice[i]);
            sorted_slice.erase(sorted_slice.begin()+i);
        }
        if(sorted_slice[i].event_type == OUT)
        {
            sub_slice.emplace_back(sorted_slice[i]);
            sorted_slice.erase(sorted_slice.begin()+i);
        }
    }
    std::sort(sub_slice.begin(), sub_slice.end());
    sorted_slice.insert(sorted_slice.begin(), sub_slice.begin(), sub_slice.end());

    return sorted_slice;
}

int CountCells(const std::deque<Event>& slice, int curr_idx)
{
    int cell_num = 0;
    for(int i = 0; i < curr_idx; i++)
    {
        if(slice[i].event_type==IN)
        {
            cell_num++;
        }
        if(slice[i].event_type==FLOOR)
        {
            cell_num++;
        }
    }
    return cell_num;
}


int CountCells2(const std::deque<Event>& slice, int curr_idx)
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




/***
 * in.y在哪个cell的ceil和floor中间，就选取哪个cell作为in operation的curr cell， in上面的点为c, in下面的点为f
 * 若out.y在cell A的上界A_c以下，在cell B的下界B_f以上，则cell A为out operation的top cell， cell B为out operation的bottom cell，out上面的点为c, out下面的点为f
 * cell A和cell B在cell slice中必需是紧挨着的，中间不能插有别的cell
 * 对于每个slice，从上往下统计有几个in或floor（都可代表某个cell的下界），每有一个就代表该slice上有几个cell完成上下界的确定。ceil或floor属于该slice上的哪个cell，就看之前完成了对几个cell
 * 的上下界的确定，则ceil和floor是属于该slice上的下一个cell。应等执行完该slice中所有的in和out事件后再统计cell数。
 */

/**
 * TODO: 1. in and out event in different obstacles in the same slice
 *       2. concave obstacle case
 *       3. several in/out event in the same obstacle
 * **/

void ExecuteCellDecomposition(std::deque<std::deque<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    int slice_x = INT_MAX;
    int event_y = INT_MAX;

    std::deque<Event> sorted_slice;

    std::vector<int> sub_cell_index_slices;

    int cell_counter = 0;

    cell_index_slice.emplace_back(0); // initialization

    for(int i = 0; i < slice_list.size(); i++)
    {
        slice_x = slice_list[i].front().x;
        slice_list[i].emplace_front(Event(INT_MAX, slice_x, 0, CEILING));       // add map upper boundary
        slice_list[i].emplace_back(Event(INT_MAX, slice_x, map.rows-1, FLOOR)); // add map lower boundary

        sorted_slice = SortSliceEvent(slice_list[i]);

        for(int j = 0; j < sorted_slice.size(); j++)
        {
            if(sorted_slice[j].event_type == IN)
            {
                if(sorted_slice.size() == 3)
                {
                    curr_cell_idx = cell_index_slice.back();
                    ExecuteOpenOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                                        Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                                        Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                    cell_index_slice.pop_back();
                    cell_index_slice.emplace_back(curr_cell_idx+1);
                    cell_index_slice.emplace_back(curr_cell_idx+2);

                    slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                    slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                    slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;
                }
                else
                {
                    event_y = sorted_slice[j].y;
                    for(int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            ExecuteOpenOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                                                Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                                                Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());

                            slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                            slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                            slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;

                            break;
                        }
                    }
                }
            }
            if(sorted_slice[j].event_type == OUT)
            {
                event_y = sorted_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(top_cell_idx, bottom_cell_idx,
                                              Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                              Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                              Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));

                        slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                        slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                        slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;

                        break;
                    }
                }

            }
            if(sorted_slice[j].event_type == CEILING)
            {
                cell_counter = CountCells(slice_list[i],sorted_slice[j].original_index_in_slice);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!slice_list[i][sorted_slice[j].original_index_in_slice].isUsed)
                {
                    ExecuteCeilOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y));
                }
            }
            if(sorted_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(slice_list[i],sorted_slice[j].original_index_in_slice);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!slice_list[i][sorted_slice[j].original_index_in_slice].isUsed)
                {
                    ExecuteFloorOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y));
                }
            }
        }
    }
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


void ExecuteCellDecomposition2(std::deque<std::deque<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

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
                        if(std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end())
                        {
                            rewrite = true;
                        } else
                        {
                            rewrite = false;
                        }

                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(curr_slice[m].y == cell_graph[cell_index_slice[k]].ceiling.back().y)
                            {
                                curr_slice[m].isUsed = true;
                                break;
                            }
                        }
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(curr_slice[n].y == cell_graph[cell_index_slice[k]].floor.back().y)
                            {
                                curr_slice[n].isUsed = true;
                                break;
                            }
                        }


                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(curr_cell_idx,
                                             Point2D(curr_slice[j].x, curr_slice[j].y),
                                             cell_graph[cell_index_slice[k]].ceiling.back(),
                                             cell_graph[cell_index_slice[k]].floor.back(),
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
                        if(std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end())
                        {
                            rewrite = true;
                        }
                        else
                        {
                            rewrite = false;
                        }

                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(curr_slice[m].y == cell_graph[cell_index_slice[k-1]].ceiling.back().y)
                            {
                                curr_slice[m].isUsed = true;
                                break;
                            }
                        }
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(curr_slice[n].y == cell_graph[cell_index_slice[k]].floor.back().y)
                            {
                                curr_slice[n].isUsed = true;
                                break;
                            }
                        }


                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];

                        ExecuteCloseOperation(top_cell_idx, bottom_cell_idx,
                                              Point2D(curr_slice[j].x, curr_slice[j].y),
                                              cell_graph[cell_index_slice[k-1]].ceiling.back(),
                                              cell_graph[cell_index_slice[k]].floor.back(),
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

                        if(std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k])==original_cell_index_slice.end())
                        {
                            rewrite = true;
                        } else
                        {
                            rewrite = false;
                        }

                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(curr_slice[m].y == cell_graph[cell_index_slice[k]].ceiling.back().y)
                            {
                                curr_slice[m].isUsed = true;
                                break;
                            }
                        }
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(curr_slice[n].y == cell_graph[cell_index_slice[k]].floor.back().y)
                            {
                                curr_slice[n].isUsed = true;
                                break;
                            }
                        }

                        curr_cell_idx = cell_index_slice[k];
                        ExecuteOpenOperation(curr_cell_idx,
                                             Point2D(curr_slice[j-1].x, curr_slice[j-1].y),  // in top
                                             Point2D(curr_slice[j].x, curr_slice[j].y),      // in bottom
                                             cell_graph[cell_index_slice[k]].ceiling.back(),       // c
                                             cell_graph[cell_index_slice[k]].floor.back(),         // f
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

                        if(std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k-1]) == original_cell_index_slice.end())
                        {
                            rewrite = true;
                        }
                        else
                        {
                            rewrite = false;
                        }

                        for(int m = 0; m < curr_slice.size(); m++)
                        {
                            if(curr_slice[m].y == cell_graph[cell_index_slice[k-1]].ceiling.back().y)
                            {
                                curr_slice[m].isUsed = true;
                                break;
                            }
                        }
                        for(int n = 0; n < curr_slice.size(); n++)
                        {
                            if(curr_slice[n].y == cell_graph[cell_index_slice[k]].floor.back().y)
                            {
                                curr_slice[n].isUsed = true;
                                break;
                            }
                        }

                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];


                        ExecuteCloseOperation(top_cell_idx, bottom_cell_idx,
                                              Point2D(curr_slice[j-1].x, curr_slice[j-1].y),   // out top
                                              Point2D(curr_slice[j].x, curr_slice[j].y),       // out bottom

                                              cell_graph[cell_index_slice[k-1]].ceiling.back(),   // c
                                              cell_graph[cell_index_slice[k]].floor.back(),       // f
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
                    if(event_y > cell_graph[cell_index_slice[k-1]].floor.back().y && event_y < cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {
                        ExecuteInnerOpenOperation(Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_in
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
                    if(event_y > cell_graph[cell_index_slice[k-1]].floor.back().y && event_y < cell_graph[cell_index_slice[k]].ceiling.back().y)
                    {

                        ExecuteInnerOpenOperation(Point2D(curr_slice[j-1].x, curr_slice[j-1].y), // inner_in_top,
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
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out
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
                    if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        curr_cell_idx = cell_index_slice[k];
                        ExecuteInnerCloseOperation(curr_cell_idx, Point2D(curr_slice[j-1].x, curr_slice[j-1].y), Point2D(curr_slice[j].x, curr_slice[j].y));  // inner_out_top, inner_out_bottom
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
                cell_counter = CountCells2(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteCeilOperation(curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
            if(curr_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells2(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteFloorOperation(curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
        }
    }
}




std::vector<Point2D> ComputeCellCornerPoints(CellNode cell, int robot_radius=0)
{
    int c_size = cell.ceiling.size();
    int f_size = cell.floor.size();

    // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
    std::vector<Point2D> corner_points = {
            Point2D(cell.ceiling[robot_radius + 1].x, cell.ceiling[robot_radius + 1].y + (robot_radius + 1)),
            Point2D(cell.floor[robot_radius + 1].x, cell.floor[robot_radius + 1].y - (robot_radius + 1)),
            Point2D(cell.floor[f_size-1-(robot_radius + 1)].x, cell.floor[f_size-1-(robot_radius + 1)].y - (robot_radius + 1)),
            Point2D(cell.ceiling[c_size-1-(robot_radius + 1)].x, cell.ceiling[c_size-1-(robot_radius + 1)].y + (robot_radius + 1))

    };
    return corner_points;
}

Point2D FindNextEntrance(Point2D curr_point, CellNode next_cell, int& corner_indicator, int robot_radius=0)
{

    double distance = DBL_MAX;
    Point2D next_entrance = Point2D(INT_MAX, INT_MAX);

    std::vector<Point2D> next_points = ComputeCellCornerPoints(next_cell, robot_radius);

    for(int i = 0; i < next_points.size(); i++)
    {
        if(std::sqrt(pow((next_points[i].x-curr_point.x),2)+pow((next_points[i].y-curr_point.y),2))<distance)
        {
            distance = std::sqrt(pow((next_points[i].x-curr_point.x),2)+pow((next_points[i].y-curr_point.y),2));
            next_entrance.x = next_points[i].x;
            next_entrance.y = next_points[i].y;
            corner_indicator = i;
        }
    }
    return next_entrance;
}

std::deque<Point2D> ExitAlongWall(Point2D start, Point2D end, int end_corner_indicator, CellNode cell, int robot_radius=0)
{
    int start_corner_indicator = INT_MAX;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell, robot_radius);
    for(int i = 0; i < corner_points.size(); i++)
    {
        if(corner_points[i].x==start.x && corner_points[i].y == start.y)
        {
            start_corner_indicator = i;
            break;
        }
    }

    std::vector<Point2D> left, bottom, right, top;  // 从上往下或从左往右
    for(int y = corner_points[0].y; y < corner_points[1].y; y++)
    {
        left.emplace_back(Point2D(corner_points[0].x, y));
    }
    for(int i = robot_radius+1; i < cell.ceiling.size()-(robot_radius+1); i++)
    {
        top.emplace_back(Point2D(cell.ceiling[i].x,cell.ceiling[i].y+(robot_radius+1)));
        bottom.emplace_back(Point2D(cell.floor[i].x,cell.floor[i].y-(robot_radius+1)));
    }
    for(int y = corner_points[3].y; y < corner_points[2].y; y++)
    {
        right.emplace_back(Point2D(corner_points[3].x, y));
    }


    std::deque<Point2D> path;

    if (start_corner_indicator == end_corner_indicator)
    {
        return path;
    }

    if(start_corner_indicator == TOPLEFT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), top.begin(), top.end());
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), left.begin(), left.end());
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(),left.begin(), left.end());
        path.insert(path.end(), bottom.begin(), bottom.end());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), top.rbegin(), top.rend());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), top.rbegin(), top.rend());
        path.insert(path.end(), left.begin(), left.end());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(), right.begin(), right.end());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), left.rbegin(), left.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), bottom.begin(), bottom.end());
        path.insert(path.end(), right.rbegin(), right.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(), bottom.begin(), bottom.end());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), right.rbegin(), right.rend());
        path.insert(path.end(), top.rbegin(), top.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), right.rbegin(), right.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), bottom.rbegin(), bottom.rend());
        return path;
    }

}

std::deque<Point2D> FindLinkingPath(Point2D curr_exit, Point2D next_entrance, CellNode curr_cell, CellNode next_cell, int robot_radius=0)
{
    std::deque<Point2D> path;
    std::deque<Point2D> sub_path;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator, robot_radius);
    sub_path = ExitAlongWall(curr_exit, exit, exit_corner_indicator, curr_cell, robot_radius);
    path.insert(path.begin(), sub_path.begin(), sub_path.end());

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

    return path;
}

std::deque<Point2D> PathIninitialization(Point2D start, CellNode cell, int robot_radius=0)
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

std::deque<cv::Scalar> JetColorMap;

void InitializeColorMap(int repeat_times)
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

void UpdateColorMap()
{
    cv::Scalar color = JetColorMap.front();
    JetColorMap.pop_front();
    JetColorMap.emplace_back(color);
}



int main() {

    map = cv::Mat::zeros(500, 500, CV_8UC3);
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

    std::vector<cv::Point> contour = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16};
    std::vector<std::vector<cv::Point>> contours = {contour};
//    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));
//
//    cv::namedWindow("map", cv::WINDOW_NORMAL);
//    cv::imshow("map", map);
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

//    std::vector<Event> event_list = InitializeEventList(polygon, 0);
//    EventTypeAllocator(event_list);

    PolygonList polygons = {polygon};
    std::vector<Event> event_list = EventListGenerator2(polygons);

    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
    InitializeCellDecomposition(Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition2(slice_list);
    FinishCellDecomposition(Point2D(slice_list.back().back().x, slice_list.back().back().y));

    Point2D start_point = Point2D(10, 10);
    int start_cell_index = 0;
    int robot_radius = 5;

    std::deque<Point2D> first_steps = PathIninitialization(start_point, cell_graph[start_cell_index], robot_radius);
    WalkingThroughGraph(start_cell_index);


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

    int repeat_times = 20;
    InitializeColorMap(repeat_times);
//    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));
//    cv::circle(map, cv::Point(start_point.x, start_point.y), 3, cv::Scalar(0, 0, 255), -1);

    cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
    cv::imshow("trajectory", map);
    cv::waitKey(0);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        map = cv::Mat::zeros(500, 500, CV_8UC3);
//        cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));
        DrawCells(cell_graph[i]);
        cv::imshow("trajectory", map);
        cv::waitKey(0);
    }




//    for(int i = 0; i < first_steps.size(); i++)
//    {
//        cv::circle(map, cv::Point(first_steps[i].x, first_steps[i].y), 1, JetColorMap.front(), -1);
//        UpdateColorMap();
//
//        cv::imshow("trajectory", map);
//        cv::waitKey(1);
//    }
//
//    std::vector<Point2D> sub_path;
//    int corner_indicator = TOPLEFT;
//
//    for(int i = path.size()-1; i >= 0; i--)
//    {
//        sub_path = GetBoustrophedonPath(path[i], corner_indicator, robot_radius);
//        for(int j = 0; j < sub_path.size(); j++)
//        {
//            cv::circle(map, cv::Point(sub_path[j].x, sub_path[j].y), 1, JetColorMap.front(), -1);
//            UpdateColorMap();
//
//            cv::imshow("trajectory", map);
//            cv::waitKey(1);
//        }
//
//        cell_graph[path[i].cellIndex].isCleaned = true;
//
//        if((i-1)>=0)
//        {
//            Point2D curr_exit = sub_path.back();
//            Point2D next_entrance = FindNextEntrance(curr_exit, path[i - 1], corner_indicator, robot_radius);
//            std::deque<Point2D> link_path = FindLinkingPath(curr_exit, next_entrance, path[i], path[i-1], robot_radius);
//            for(int k = 0; k < link_path.size(); k++)
//            {
//                cv::circle(map, cv::Point(link_path[k].x, link_path[k].y), 1, JetColorMap.front(), -1);
//                UpdateColorMap();
//
//                cv::imshow("trajectory", map);
//                cv::waitKey(1);
//            }
//        }
//    }
//    cv::waitKey(0);









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







//    map = cv::Mat::zeros(400, 400, CV_8UC3);
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
//
//    PolygonList polygons = {polygon1, polygon2};
//    std::vector<Event> event_list = EventListGenerator(polygons);
//
//
//    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
//    InitializeCellDecomposition(Point2D(slice_list.front().front().x, slice_list.front().front().y));
//    ExecuteCellDecomposition(slice_list);
//    FinishCellDecomposition(Point2D(slice_list.back().back().x, slice_list.back().back().y));
//
//    Point2D start_point = Point2D(150, 100);
//    int start_cell_index = 1;
//    int robot_radius = 5;
//
//    std::deque<Point2D> first_steps = PathIninitialization(start_point, cell_graph[start_cell_index], robot_radius);
//    WalkingThroughGraph(start_cell_index);
//
//
//    for(int i = 0; i < cell_graph.size(); i++)
//    {
//        std::cout<<"cell "<<i<<" 's ceiling points number:" << cell_graph[i].ceiling.size()<<std::endl;
//        std::cout<<"cell "<<i<<" 's floor points number:" << cell_graph[i].floor.size()<<std::endl;
//    }
//
//    std::cout<<cell_graph.size()<<std::endl;
//
//
//    for(int i = 0; i < cell_graph.size(); i++)
//    {
//        for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
//        {
//            std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
//        }
//    }
//
//    int repeat_times = 20;
//    InitializeColorMap(repeat_times);
//
//    std::vector<cv::Point> contour1 = {cv::Point(200,300), cv::Point(300,200), cv::Point(200,100), cv::Point(100,200)};
//    std::vector<cv::Point> contour2 = {cv::Point(300,350), cv::Point(350,300), cv::Point(300,250), cv::Point(250,300)};
//    std::vector<std::vector<cv::Point>> contours = {contour1, contour2};
//    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));
//    cv::circle(map, cv::Point(start_point.x, start_point.y), 3, cv::Scalar(0, 0, 255), -1);
//
//    cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
//    cv::imshow("trajectory", map);
//    cv::waitKey(0);
//
//    for(int i = 0; i < cell_graph.size(); i++)
//    {
//        DrawCells(cell_graph[i]);
//        cv::imshow("trajectory", map);
//        cv::waitKey(500);
//    }
//
//    for(int i = 0; i < first_steps.size(); i++)
//    {
//        cv::circle(map, cv::Point(first_steps[i].x, first_steps[i].y), 1, JetColorMap.front(), -1);
//        UpdateColorMap();
//
//        cv::imshow("trajectory", map);
//        cv::waitKey(1);
//    }
//
//    std::vector<Point2D> sub_path;
//    int corner_indicator = TOPLEFT;
//
//    for(int i = path.size()-1; i >= 0; i--)
//    {
//        sub_path = GetBoustrophedonPath(path[i], corner_indicator, robot_radius);
//        for(int j = 0; j < sub_path.size(); j++)
//        {
//            cv::circle(map, cv::Point(sub_path[j].x, sub_path[j].y), 1, JetColorMap.front(), -1);
//            UpdateColorMap();
//
//            cv::imshow("trajectory", map);
//            cv::waitKey(1);
//        }
//
//        cell_graph[path[i].cellIndex].isCleaned = true;
//
//        if((i-1)>=0)
//        {
//            Point2D curr_exit = sub_path.back();
//            Point2D next_entrance = FindNextEntrance(curr_exit, path[i - 1], corner_indicator, robot_radius);
//            std::deque<Point2D> link_path = FindLinkingPath(curr_exit, next_entrance, path[i], path[i-1], robot_radius);
//            for(int k = 0; k < link_path.size(); k++)
//            {
//                cv::circle(map, cv::Point(link_path[k].x, link_path[k].y), 1, JetColorMap.front(), -1);
//                UpdateColorMap();
//
//                cv::imshow("trajectory", map);
//                cv::waitKey(1);
//            }
//        }
//    }
//    cv::waitKey(0);

    return 0;
}