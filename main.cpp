#include <iostream>
#include <vector>
#include <deque>
#include <list>
#include <algorithm>

enum EventType
{
    IN,
    OUT,
    CEILING,
    FLOOR
};

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

typedef std::vector<Point2D> Polygon;
typedef std::vector<Polygon> PolygonList;
typedef std::vector<Point2D> Edge;
typedef std::vector<Edge> EdgeList;

class Event
{
public:
    Event(int x_pos, int y_pos, EventType type)
    {
        x = x_pos;
        y = y_pos;
        event_type = type;
    }

    int x;
    int y;
    Edge ceiling_edge;
    Edge floor_edge;
    EventType event_type;
};

class CellNode
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        derivedFrom = nullptr;
    }
    bool isVisited;
    bool isCleaned;
    EdgeList ceiling;
    EdgeList floor;

    CellNode* derivedFrom;
    std::vector<CellNode*> neighbor_cells;

    int cellIndex;
};



std::deque<CellNode> path;

void DepthFirstSearch(CellNode& cell)
{
    cell.isVisited = true;
    path.emplace_front(cell);
    std::cout<< "cell: " <<cell.cellIndex<<std::endl;

    CellNode* neighbor = nullptr;

    for(int i = 0; i < cell.neighbor_cells.size(); i++)
    {
        neighbor = cell.neighbor_cells[i];
        if(!neighbor->isVisited)
        {
            break;
        }
    }

    if(!neighbor->isVisited) // unvisited neighbor found
    {
        neighbor->derivedFrom = &cell;
        DepthFirstSearch(*neighbor);
    }
    else  // unvisited neighbor not found
    {

        if (cell.derivedFrom == nullptr) // cannot go on back-tracking
        {
            return;
        }
        else
        {
            DepthFirstSearch(*cell.derivedFrom);
        }
    }
}


std::vector<Point2D> GetBoustrophedonPath(std::vector<Point2D> ceiling, std::vector<Point2D> floor, int robot_radius=0)
{
    std::vector<Point2D> path;

    int x=0, y=0, y_start=0, y_end=0;
    bool reverse = false;

    for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i++)
    {
        x = ceiling[i].x;

        if(!reverse)
        {
            y_start = ceiling[i].y + (robot_radius + 1);
            y_end   = floor[i].y - (robot_radius + 1);

            for(y = y_start; y <= y_end; y++)
            {
                path.emplace_back(Point2D(x, y));
                std::cout<< x << "," << y << std::endl;
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
                std::cout<< x << "," << y << std::endl;
            }
            reverse = !reverse;
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
    return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y));
}


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

        event_list.emplace_back(Event(polygon[leftmost_idx].x, polygon[leftmost_idx].y, IN));
        event_list.emplace_back(Event(polygon[rightmost_idx].x, polygon[rightmost_idx].y, OUT));

        // 假设多边形为凸壳 且各个顶点按照逆时针的顺序排列
        if (leftmost_idx < rightmost_idx)
        {
            for(int m = 0; m < polygon.size(); m++)
            {
                if(leftmost_idx < m && m < rightmost_idx)
                {
                    event_list.emplace_back(Event(polygon[m].x, polygon[m].y, CEILING));
                }
                if(m < leftmost_idx || m >rightmost_idx)
                {
                    event_list.emplace_back(Event(polygon[m].x, polygon[m].y, FLOOR));
                }
            }
        }
        else if(leftmost_idx > rightmost_idx)
        {
            for(int n = 0; n < polygon.size(); n++)
            {
                if(rightmost_idx < n && n < leftmost_idx)
                {
                    event_list.emplace_back(Event(polygon[n].x, polygon[n].y, FLOOR));
                }
                if(n < rightmost_idx || n > leftmost_idx)
                {
                    event_list.emplace_back(Event(polygon[n].x, polygon[n].y, CEILING));
                }
            }
        }
    }
    std::sort(event_list.begin(),event_list.end());
    return event_list;
}



void CellDecomposition()
{
    return;
}



int main() {
//    test GetBoustrophedonPath
//    std::vector<Point2D> ceil = {Point2D(0,0),Point2D(1,0),Point2D(2,0),Point2D(3,0),Point2D(4,0)};
//    std::vector<Point2D> floor = {Point2D(0,4),Point2D(1,4),Point2D(2,4),Point2D(3,4),Point2D(4,4)};
//
//    std::vector<Point2D> path = GetBoustrophedonPath(ceil, floor);
//    std::vector<Point2D>::iterator it = path.begin();

//    test DepthFirstSearch
//    CellNode cell1,cell2,cell3,cell4,cell5,cell6,cell7,cell8,cell9,cell10,cell11;
//    cell1.cellIndex=1;
//    cell2.cellIndex=2;
//    cell3.cellIndex=3;
//    cell4.cellIndex=4;
//    cell5.cellIndex=5;
//    cell6.cellIndex=6;
//    cell7.cellIndex=7;
//    cell8.cellIndex=8;
//    cell9.cellIndex=9;
//    cell10.cellIndex=10;
//    cell11.cellIndex=11;
//
//    cell1.neighbor_cells={&cell2,&cell3};
//    cell2.neighbor_cells={&cell4,&cell5,&cell1};
//    cell3.neighbor_cells={&cell6,&cell7,&cell1};
//    cell4.neighbor_cells={&cell8,&cell9,&cell2};
//    cell5.neighbor_cells={&cell2};
//    cell6.neighbor_cells={&cell3};
//    cell7.neighbor_cells={&cell10,&cell11,&cell3};
//    cell8.neighbor_cells={&cell4};
//    cell9.neighbor_cells={&cell4};
//    cell10.neighbor_cells={&cell7};
//    cell11.neighbor_cells={&cell7};
//
//    DepthFirstSearch(cell1);

//    test EventListGenerator
//    Polygon polygon = {Point2D(2,3), Point2D(4,3), Point2D(5,2), Point2D(3,1), Point2D(1,2)};
//    PolygonList polygons = {polygon};
//
//    std::vector<Event> event_list = EventListGenerator(polygons);
//
//    for(auto it : event_list)
//    {
//        std::cout<< "x:" << it.x << ", y:"<< it.y << ", type:" << it.event_type << std::endl;
//    }

    return 0;
}