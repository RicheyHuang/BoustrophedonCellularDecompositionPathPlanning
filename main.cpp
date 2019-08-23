#include <iostream>
#include <vector>
#include <deque>
#include <list>

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

typedef std::vector<Point2D> Edge;
typedef std::vector<Edge> Edges;

class Event
{
public:
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
    Edges ceiling;
    Edges floor;

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





int main() {
//    std::vector<Point2D> ceil = {Point2D(0,0),Point2D(1,0),Point2D(2,0),Point2D(3,0),Point2D(4,0)};
//    std::vector<Point2D> floor = {Point2D(0,4),Point2D(1,4),Point2D(2,4),Point2D(3,4),Point2D(4,4)};
//
//    std::vector<Point2D> path = GetBoustrophedonPath(ceil, floor);
//    std::vector<Point2D>::iterator it = path.begin();

    CellNode cell1,cell2,cell3,cell4,cell5,cell6,cell7,cell8,cell9,cell10,cell11;
    cell1.cellIndex=1;
    cell2.cellIndex=2;
    cell3.cellIndex=3;
    cell4.cellIndex=4;
    cell5.cellIndex=5;
    cell6.cellIndex=6;
    cell7.cellIndex=7;
    cell8.cellIndex=8;
    cell9.cellIndex=9;
    cell10.cellIndex=10;
    cell11.cellIndex=11;



    cell1.neighbor_cells={&cell2,&cell3};
    cell2.neighbor_cells={&cell4,&cell5,&cell1};
    cell3.neighbor_cells={&cell6,&cell7,&cell1};
    cell4.neighbor_cells={&cell8,&cell9,&cell2};
    cell5.neighbor_cells={&cell2};
    cell6.neighbor_cells={&cell3};
    cell7.neighbor_cells={&cell10,&cell11,&cell3};
    cell8.neighbor_cells={&cell4};
    cell9.neighbor_cells={&cell4};
    cell10.neighbor_cells={&cell7};
    cell11.neighbor_cells={&cell7};

    DepthFirstSearch(cell1);


    return 0;
}