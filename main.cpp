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

void WalkingThroughGraph(int cell_index)  // Depth First Search Method
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

std::deque<Point2D> GetBoustrophedonPath(CellNode cell, int corner_indicator, int robot_radius=0)
{
    std::vector<Point2D> ceiling = cell.ceiling;
    std::vector<Point2D> floor = cell.floor;

    std::deque<Point2D> path;

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

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i = i + (robot_radius+1))
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
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
//                            if(floor[i+j+(robot_radius + 1)].x == floor[i+j+(robot_radius + 1)+1].x)
//                            {
//                                if (floor[i + j + (robot_radius + 1)].y > floor[i + j + (robot_radius + 1) + 1].y)
//                                {
//                                    i = i - (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(floor[i+j].x == floor[i+j+1].x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
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
                            if(x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
//                            if(ceiling[i+j+(robot_radius + 1)].x == ceiling[i+j+(robot_radius + 1)+1].x)
//                            {
//                               if(ceiling[i+j+(robot_radius + 1)].y < ceiling[i+j+(robot_radius + 1)+1].y)
//                               {
//                                   i = i - (robot_radius - (j - 1));
//                                   break;
//                               }
//                            }
//                            if(ceiling[i+j].x == ceiling[i+j+1].x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }

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

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-(robot_radius+1))
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
                            if(x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
//                            if(floor[i-j-(robot_radius + 1)].x == floor[i-j-(robot_radius + 1)-1].x)
//                            {
//                                if(floor[i-j-(robot_radius + 1)].y > floor[i-j-(robot_radius + 1)-1].y)
//                                {
//                                    i = i + (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(floor[i-j].x == floor[i-j-1].x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
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
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
//                            if(ceiling[i-j-(robot_radius + 1)].x == ceiling[i-j-(robot_radius + 1)-1].x)
//                            {
//                                if(ceiling[i-j-(robot_radius + 1)].y < ceiling[i-j-(robot_radius + 1)-1].y)
//                                {
//                                    i = i + (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(ceiling[i-j].x == ceiling[i-j-1].x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
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

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i=i+(robot_radius+1))
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
                            if(x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
//                            if(ceiling[i+j+(robot_radius + 1)].x == ceiling[i+j+(robot_radius + 1)+1].x)
//                            {
//                                if(ceiling[i+j+(robot_radius + 1)].y > ceiling[i+j+(robot_radius + 1)+1].y)
//                                {
//                                    i = i - (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(ceiling[i+j].x == ceiling[i+j+1].x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
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
                            if(x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                i = i - (robot_radius - (j - 1));
                                break;
                            }
//                            if(floor[i+j+(robot_radius + 1)].x == floor[i+j+(robot_radius + 1)+1].x)
//                            {
//                                if(floor[i+j+(robot_radius + 1)].y < floor[i+j+(robot_radius + 1)+1].y)
//                                {
//                                    i = i - (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(floor[i+j].x == floor[i+j+1].x)
//                            {
//                                i = i - (robot_radius - (j - 1));
//                                break;
//                            }
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

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-(robot_radius+1))
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
                            if(x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
//                            if(ceiling[i-j-(robot_radius + 1)].x == ceiling[i-j-(robot_radius + 1)-1].x)
//                            {
//                                if(ceiling[i-j-(robot_radius + 1)].y < ceiling[i-j-(robot_radius + 1)-1].y)
//                                {
//                                    i = i + (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(ceiling[i-j].x == ceiling[i-j-1].x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
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
                            if(x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                i = i + (robot_radius - (j - 1));
                                break;
                            }
//                            if(floor[i-j-(robot_radius + 1)].x == floor[i-j-(robot_radius + 1)-1].x)
//                            {
//                                if(floor[i-j-(robot_radius + 1)].y > floor[i-j-(robot_radius + 1)-1].y)
//                                {
//                                    i = i + (robot_radius - (j - 1));
//                                    break;
//                                }
//                            }
//                            if(floor[i-j].x == floor[i-j-1].x)
//                            {
//                                i = i + (robot_radius - (j - 1));
//                                break;
//                            }
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
    int temp_index;
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
std::vector<Event> EventListGenerator(PolygonList polygons)
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
        map.at<cv::Vec3b>(cell.ceiling[i].y, cell.ceiling[i].x) = cv::Vec3b(96, 96, 96); // 96 96 96
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        map.at<cv::Vec3b>(cell.floor[i].y, cell.floor[i].x) = cv::Vec3b(96, 96, 96);
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), cv::Scalar(96, 96, 96));
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), cv::Scalar(96, 96, 96));
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
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
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
                    if(event_y >= cell_graph[cell_index_slice[k-1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
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
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
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
                    if(event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
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
                cell_counter = CountCells(curr_slice,j);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!curr_slice[j].isUsed)
                {
                    ExecuteCeilOperation(curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                }
            }
            if(curr_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(curr_slice,j);
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
    Point2D next_entrance = Point2D(INT_MAX, INT_MAX);

    int front_x = next_cell.ceiling.front().x;
    int back_x = next_cell.ceiling.back().x;

    if(abs(curr_point.x - front_x) < abs(curr_point.x - back_x))
    {
        if(abs(curr_point.y - next_cell.ceiling.front().y)<abs(curr_point.y - next_cell.floor.front().y))
        {
            next_entrance.x = next_cell.ceiling[robot_radius + 1].x;
            next_entrance.y = next_cell.ceiling[robot_radius + 1].y + (robot_radius + 1);
            corner_indicator = TOPLEFT;
        }
        else
        {
            next_entrance.x = next_cell.floor[robot_radius + 1].x;
            next_entrance.y = next_cell.floor[robot_radius + 1].y - (robot_radius + 1);
            corner_indicator = BOTTOMLEFT;
        }
    }
    else
    {
        if(abs(curr_point.y - next_cell.ceiling.back().y)<abs(curr_point.y - next_cell.floor.back().y))
        {
            next_entrance.x = next_cell.ceiling[next_cell.ceiling.size()-1-(robot_radius + 1)].x;
            next_entrance.y = next_cell.ceiling[next_cell.ceiling.size()-1-(robot_radius + 1)].y + (robot_radius + 1);
            corner_indicator = TOPRIGHT;
        }
        else
        {
            next_entrance.x = next_cell.floor[next_cell.floor.size()-1-(robot_radius + 1)].x;
            next_entrance.y = next_cell.floor[next_cell.floor.size()-1-(robot_radius + 1)].y - (robot_radius + 1);
            corner_indicator = BOTTOMRIGHT;
        }
    }

    return next_entrance;
}

std::deque<Point2D> ExitAlongWall(Point2D start, Point2D& end, CellNode cell, int robot_radius=0)
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
        top.emplace_back(Point2D(cell.ceiling[i].x,cell.ceiling[i].y+(robot_radius+1)));
        bottom.emplace_back(Point2D(cell.floor[i].x,cell.floor[i].y-(robot_radius+1)));
    }


    std::deque<Point2D> path;

    if (start_corner_indicator == end_corner_indicator)
    {
        return path;
    }

    if(start_corner_indicator == TOPLEFT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.end(), top.begin(), top.end());
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
        path.insert(path.end(), top.begin(), top.end());
        end.x = corner_points[TOPRIGHT].x;
        end.y = corner_points[TOPRIGHT].y;
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.end(), top.rbegin(), top.rend());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.end(), top.rbegin(), top.rend());
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
        path.insert(path.end(), bottom.begin(), bottom.end());
        end.x = corner_points[BOTTOMRIGHT].x;
        end.y = corner_points[BOTTOMRIGHT].y;
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.end(), bottom.begin(), bottom.end());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.end(), bottom.rbegin(), bottom.rend());
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
        path.insert(path.end(), bottom.rbegin(), bottom.rend());
        return path;
    }

}

std::deque<Point2D> FindLinkingPath(Point2D curr_exit, Point2D& next_entrance, int& corner_indicator, CellNode curr_cell, CellNode next_cell, int robot_radius=0)
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

int DetermineCellIndex(Point2D point)
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

std::deque<int> FindShortestPath(Point2D start, Point2D end)
{
    int start_cell_index = DetermineCellIndex(start);
    int end_cell_index = DetermineCellIndex(end);

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


std::deque<Point2D> WalkingInsideCell(CellNode cell, Point2D start, Point2D end, int robot_radius=0)
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


std::deque<Point2D> WalkingCrossCells(std::deque<int> cell_path, Point2D start, Point2D end, int robot_radius=0)
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
        sub_path = GetBoustrophedonPath(cells[cell_path[i]], curr_corner_indicator, robot_radius);
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


int main() {

//    map = cv::Mat::zeros(500, 500, CV_8UC3);
    map = cv::Mat::zeros(600, 600, CV_8UC3);

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
//    cv::Point p1(125,125),
//              p2(125,175),
//              p3(225,175),
//              p4(225,225),
//              p5(175,250),
//              p6(225,300),
//              p7(125,325),
//              p8(125,375),
//              p9(375,375),
//              p10(375,325),
//              p11(275,325),
//              p12(275,275),
//              p13(325,250),
//              p14(275,200),
//              p15(375,175),
//              p16(375,125);

    cv::Point p1(100,100),
            p2(100,500),
            p3(150,500),
            p4(150,150),
            p5(450,150),
            p6(450,300),
            p7(300,300),
            p8(300,250),
            p9(350,250),
            p10(350,200),
            p11(250,200),
            p12(250,350),
            p13(500,350),
            p14(500,100);


//    std::vector<cv::Point> contour = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16};
    std::vector<cv::Point> contour = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14};

    std::vector<std::vector<cv::Point>> contours = {contour};
    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));

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


    std::vector<Event> event_list = EventListGenerator(polygons);

    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
    InitializeCellDecomposition(Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition(slice_list);
    FinishCellDecomposition(Point2D(slice_list.back().back().x, slice_list.back().back().y));

    Point2D start_point = Point2D(10, 10);
    Point2D end_point;
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

    int repeat_times = 30;
    InitializeColorMap(repeat_times);

    cv::circle(map, cv::Point(start_point.x, start_point.y), 3, cv::Scalar(0, 0, 255), -1);

    cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
    cv::imshow("trajectory", map);
//    cv::waitKey(0);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        DrawCells(cell_graph[i]);
        cv::imshow("trajectory", map);
        cv::waitKey(500);
    }


    for(int i = 0; i < first_steps.size(); i++)
    {
        map.at<cv::Vec3b>(first_steps[i].y, first_steps[i].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
        UpdateColorMap();
        cv::imshow("trajectory", map);
        cv::waitKey(1);
    }

    std::deque<Point2D> sub_path;
    int corner_indicator = TOPLEFT;

    for(int i = path.size()-1; i >= 0; i--)
    {
        sub_path = GetBoustrophedonPath(path[i], corner_indicator, robot_radius);
        for(int j = 0; j < sub_path.size(); j++)
        {
            map.at<cv::Vec3b>(sub_path[j].y, sub_path[j].x)=cv::Vec3b(JetColorMap.front()[0],JetColorMap.front()[1],JetColorMap.front()[2]);
            UpdateColorMap();
            cv::imshow("trajectory", map);
            cv::waitKey(1);
        }

        cell_graph[path[i].cellIndex].isCleaned = true;

        if((i-1)>=0)
        {
            Point2D curr_exit = sub_path.back();
            Point2D next_entrance = FindNextEntrance(curr_exit, path[i - 1], corner_indicator, robot_radius);
            std::deque<Point2D> link_path = FindLinkingPath(curr_exit, next_entrance, corner_indicator, path[i], path[i-1], robot_radius);
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
    std::deque<int> return_cell_path = FindShortestPath(end_point, start_point);
    std::deque<Point2D> return_path = WalkingCrossCells(return_cell_path, end_point, start_point, robot_radius);
    for(int i = 0; i < return_path.size(); i++)
    {
        map.at<cv::Vec3b>(return_path[i].y, return_path[i].x)=cv::Vec3b(255, 255, 255);
//        UpdateColorMap();
        cv::imshow("trajectory", map);
        cv::waitKey(1);
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

    return 0;
}