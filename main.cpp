#include <iostream>
#include <vector>
#include <deque>
#include <list>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat map;

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

typedef std::vector<Point2D> Polygon;   // contour points extracted from a blob, sorted by counter clockwise manner
typedef std::vector<Polygon> PolygonList;
typedef std::vector<Point2D> Edge;
typedef std::vector<Edge> EdgeList;

class Event
{
public:
    Event(int obstacle_idx, int x_pos, int y_pos, EventType type)
    {
        obstacle_index = obstacle_idx;
        x = x_pos;
        y = y_pos;
        event_type = type;
    }

    int x;
    int y;
    int obstacle_index;
    Edge ceiling_edge;
    Edge floor_edge;
    EventType event_type;
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

void WalkingThroughGraph(int cell_index)
{
    cell_graph[cell_index].isVisited = true;
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
        neighbor.parentIndex = cell_graph[cell_index].cellIndex;
        WalkingThroughGraph(neighbor_idx);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else
        {
            WalkingThroughGraph(cell_graph[cell_index].parentIndex);
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


std::vector<Event> EventListGenerator(PolygonList polygons) // 多边形按照左顶点x从小到大的顺序排列
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

std::vector<std::vector<Event>> SliceListGenerator(std::vector<Event> event_list)
{
    std::vector<std::vector<Event>> slice_list;
    std::vector<Event> slice;
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

void ExecuteOpenOperation(int curr_cell_idx, Point2D in, Point2D c, Point2D f) // in event  add two new node
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(f);

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

// top cell和bottom cell按前后排列
void ExecuteCloseOperation(int top_cell_idx, int bottom_cell_idx, Point2D out, Point2D c, Point2D f) // out event  add one new node
{
    CellNode new_cell;

    new_cell.ceiling.emplace_back(c);
    new_cell.floor.emplace_back(f);

    int new_cell_idx = cell_graph.size();
    new_cell.cellIndex = new_cell_idx;

    cell_graph.emplace_back(new_cell);


    cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

    cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
    cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);

}

void ExecuteCeilOperation(int curr_cell_idx, const Point2D& ceil_point) // finish constructing last ceiling edge
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void ExecuteFloorOperation(int curr_cell_idx, const Point2D& floor_point) // finish constructing last floor edge
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}


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
}



void drawing_test(const CellNode& cell)
{
    for(int i = 0; i < cell.ceiling.size(); i++)
    {
        map.at<cv::Vec3f>(cell.ceiling[i].y, cell.ceiling[i].x) = cv::Vec3f(0, 0, 255);
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        map.at<cv::Vec3f>(cell.floor[i].y, cell.floor[i].x) = cv::Vec3f(0, 0, 255);
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), cv::Scalar(0,0,255));
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), cv::Scalar(0,0,255));
}


std::vector<int> cell_index_slices;

/***
 * in.y在哪个cell的ceil和floor中间，就选取哪个cell作为in operation的curr cell， in上面的点为c, in下面的点为f
 * 若out.y在cell A的上界A_c以下，在cell B的下界以上B_f，则cell A为out operation的top cell， cell B为out operation的bottom cell，A_c为c, B_f为f
 * cell A和cell B在cell slice中必需是紧挨着的，中间不能插有别的cell
 * 对于一个event slice分别在最前和最后补上最上的c和最下的f后，从上往下遍历该event slice，将是一对一对的cf，按顺序将每一对cf赋给cell slice从上往下的每个cell
 */

void ExecuteCellDecomposition(std::vector<std::vector<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    for(int i = 0; i < slice_list.size(); i++)
    {
        for(int j = 0; j < slice_list[i].size(); j++)
        {
            if(slice_list[i].size() == 1)
            {
                if(slice_list[i][j].event_type == IN)
                {

                }
                if(slice_list[i][j].event_type == OUT)
                {}
                if(slice_list[i][j].event_type == CEILING)
                {}
                if(slice_list[i][j].event_type == FLOOR)
                {}
            }
            else
            {
                if(slice_list[i][j].event_type == IN)
                {}
                if(slice_list[i][j].event_type == OUT)
                {}
                if(slice_list[i][j].event_type == CEILING)
                {}
                if(slice_list[i][j].event_type == FLOOR)
                {}
            }

        }
    }

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

//    test simple cell decomposition
//    map = cv::Mat::zeros(400, 400, CV_32FC3);
//    Point2D in = Point2D(100,200), c = Point2D(100,0), f = Point2D(100,399);
//    Point2D out = Point2D(300,200), c_=Point2D(300,0), f_=Point2D(300,399);
//    Point2D c_end = Point2D(399, 0), f_end = Point2D(399, 399);
//
//    CellNode cell0;
//    int cell0_idx = 0;
//    cell0.cellIndex = cell0_idx;
//    // 初始化最初的ceil和floor点
//    cell0.ceiling.emplace_back(Point2D(0,0));
//    cell0.floor.emplace_back(Point2D(0,399));
//
//    for(int i = 0; i < in.x; i++)
//    {
//        cell0.ceiling.emplace_back(Point2D(i,0));
//        cell0.floor.emplace_back(Point2D(i,map.rows-1));
//    }
//
//    cell_graph.emplace_back(cell0);
//
//    ExecuteOpenOperation(cell0_idx, in, c, f);
//    int cell1_idx = cell_graph.size()-2;
//    int cell2_idx = cell_graph.size()-1;
//
//    cv::LineIterator floor_edge1(map, cv::Point(in.x, in.y), cv::Point(200, 100), 8, true);
//    cv::LineIterator floor_edge2(map, cv::Point(200, 100), cv::Point(out.x, out.y), 8, true);
//    cv::LineIterator ceil_edge1(map, cv::Point(in.x, in.y), cv::Point(200, 300), 8, true);
//    cv::LineIterator ceil_edge2(map, cv::Point(200, 300), cv::Point(out.x, out.y), 8, true);
//
//    for(int i = 1; i < floor_edge1.count; i++)
//    {
//        ExecuteCeilOperation(cell1_idx, Point2D(floor_edge1.pos().x, 0));
//        ExecuteFloorOperation(cell1_idx, Point2D(floor_edge1.pos().x, floor_edge1.pos().y));
//        floor_edge1++;
//    }
//
//    for(int i = 1; i < floor_edge2.count; i++)
//    {
//        ExecuteCeilOperation(cell1_idx, Point2D(floor_edge2.pos().x, 0));
//        ExecuteFloorOperation(cell1_idx, Point2D(floor_edge2.pos().x, floor_edge2.pos().y));
//        floor_edge2++;
//    }
//
//    for(int i = 1; i < ceil_edge1.count; i++)
//    {
//        ExecuteCeilOperation(cell2_idx, Point2D(ceil_edge1.pos().x, ceil_edge1.pos().y));
//        ExecuteFloorOperation(cell2_idx, Point2D(ceil_edge1.pos().x, map.rows-1));
//        ceil_edge1++;
//    }
//
//    for(int i = 1; i < ceil_edge2.count; i++)
//    {
//        ExecuteCeilOperation(cell2_idx, Point2D(ceil_edge2.pos().x, ceil_edge2.pos().y));
//        ExecuteFloorOperation(cell2_idx, Point2D(ceil_edge2.pos().x, map.rows-1));
//        ceil_edge2++;
//    }
//
//
//    ExecuteCloseOperation(cell1_idx, cell2_idx, out, c_, f_);
//    int cell3_idx = cell_graph.size()-1;
//
//
//    // 封闭最后的ceil点和floor点
//    for(int i = out.x + 1; i <= map.cols-1; i++)
//    {
//        cell_graph[cell3_idx].ceiling.emplace_back(Point2D(i, 0));
//        cell_graph[cell3_idx].floor.emplace_back(Point2D(i, map.rows-1));
//    }
//
//
//    drawing_test(cell_graph[cell0_idx]);
//    cv::imshow("cells", map);
//    cv::waitKey(0);
//
//    drawing_test(cell_graph[cell1_idx]);
//    cv::imshow("cells", map);
//    cv::waitKey(0);
//
//    drawing_test(cell_graph[cell2_idx]);
//    cv::imshow("cells", map);
//    cv::waitKey(0);
//
//    drawing_test(cell_graph[cell3_idx]);
//    cv::imshow("cells", map);
//    cv::waitKey(0);
//
//    cv::imshow("cells", map);
//    cv::waitKey(0);
//
//    for(int i = 0; i < cell_graph[0].neighbor_indices.size(); i++)
//    {
//        std::cout<<"cell0's neighbor: cell "<<cell_graph[cell_graph[0].neighbor_indices[i]].cellIndex<<std::endl;
//    }
//
//    for(int i = 0; i < cell_graph[1].neighbor_indices.size(); i++)
//    {
//        std::cout<<"cell1's neighbor: cell "<<cell_graph[cell_graph[1].neighbor_indices[i]].cellIndex<<std::endl;
//    }
//
//    for(int i = 0; i < cell_graph[2].neighbor_indices.size(); i++)
//    {
//        std::cout<<"cell2's neighbor: cell "<<cell_graph[cell_graph[2].neighbor_indices[i]].cellIndex<<std::endl;
//    }
//
//    for(int i = 0; i < cell_graph[3].neighbor_indices.size(); i++)
//    {
//        std::cout<<"cell3's neighbor: cell "<<cell_graph[cell_graph[3].neighbor_indices[i]].cellIndex<<std::endl;
//    }
//
//    WalkingThroughGraph(cell0_idx);
//

//  test SliceListGenerator
//    Event e1 = Event(1, 10, 20, IN);
//    Event e2 = Event(2, 10, 30, IN);
//    Event e3 = Event(3, 20, 20, IN);
//    Event e4 = Event(4, 30, 20, IN);
//    Event e5 = Event(5, 30, 30, IN);
//    Event e6 = Event(6, 40, 20, IN);
//
//    std::vector<Event> event_list = {e1, e2, e3, e4, e5, e6};
//    std::vector<std::vector<Event>> slice_list = SliceListGenerator(event_list);
//
//    std::cout<< "slice num: "<< slice_list.size() << std::endl;
//    for(int i = 0; i < slice_list.size(); i++)
//    {
//        for(int j = 0; j < slice_list[i].size(); j++)
//        {
//            std::cout<< "slice" << i << ": event " << slice_list[i][j].obstacle_index << std::endl;
//        }
//    }


    return 0;
}