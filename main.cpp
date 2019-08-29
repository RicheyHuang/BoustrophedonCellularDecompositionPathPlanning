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

    unvisited_counter = cell_graph.size();
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


std::vector<int> cell_index_slice; // 按y从小到大排列

/***
 * in.y在哪个cell的ceil和floor中间，就选取哪个cell作为in operation的curr cell， in上面的点为c, in下面的点为f
 * 若out.y在cell A的上界A_c以下，在cell B的下界B_f以上，则cell A为out operation的top cell， cell B为out operation的bottom cell，A_c为c, B_f为f
 * cell A和cell B在cell slice中必需是紧挨着的，中间不能插有别的cell
 * 对于每个slice，从上往下统计有几个in或out或floor（都可代表某个cell的下界），每有一个就代表该slice上有几个cell完成上下界的确定。ceil或floor属于该slice上的哪个cell，就看之前完成了对几个cell
 * 的上下界的确定，则ceil和floor是属于该slice上的下一个cell
 */

void ExecuteCellDecomposition(std::deque<std::deque<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    int slice_x = INT_MAX;
    int event_y = INT_MAX;

    std::vector<int> sub_cell_index_slices;

    int cell_counter = 0;

    cell_index_slice.emplace_back(0); // initialization

    for(int i = 0; i < slice_list.size(); i++)
    {
        cell_counter = 0;

        slice_x = slice_list[i].front().x;
        slice_list[i].emplace_front(Event(INT_MAX, slice_x, 0, CEILING));       // add map upper boundary
        slice_list[i].emplace_back(Event(INT_MAX, slice_x, map.rows-1, FLOOR)); // add map lower boundary

        for(int j = 0; j < slice_list[i].size(); j++)
        {
            if(slice_list[i][j].event_type == IN)
            {
                cell_counter++;
                if(slice_list[i].size() == 3)
                {
                    curr_cell_idx = cell_index_slice.back();
                    ExecuteOpenOperation(curr_cell_idx, Point2D(slice_list[i][j].x, slice_list[i][j].y),
                                                        Point2D(slice_list[i][j-1].x, slice_list[i][j-1].y),
                                                        Point2D(slice_list[i][j+1].x, slice_list[i][j+1].y));
                    cell_index_slice.pop_back();
                    cell_index_slice.emplace_back(curr_cell_idx+1);
                    cell_index_slice.emplace_back(curr_cell_idx+2);
                }
                else
                {
                    event_y = slice_list[i][j].y;
                    for(int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            ExecuteOpenOperation(curr_cell_idx, Point2D(slice_list[i][j].x, slice_list[i][j].y),
                                                                Point2D(slice_list[i][j-1].x, slice_list[i][j-1].y),
                                                                Point2D(slice_list[i][j+1].x, slice_list[i][j+1].y));
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-1), int(cell_graph.size()-2)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                            break;
                        }
                    }
                }
            }
            if(slice_list[i][j].event_type == OUT)
            {
                cell_counter++;
                event_y = slice_list[i][j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(top_cell_idx, bottom_cell_idx,
                                              Point2D(slice_list[i][j].x, slice_list[i][j].y),
                                              cell_graph[top_cell_idx].ceiling.back(),
                                              cell_graph[bottom_cell_idx].floor.back());
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));
                        break;
                    }
                }

            }
            if(slice_list[i][j].event_type == CEILING)
            {
                curr_cell_idx = cell_index_slice[cell_counter];
                ExecuteCeilOperation(curr_cell_idx, Point2D(slice_list[i][j].x, slice_list[i][j].y));
            }
            if(slice_list[i][j].event_type == FLOOR)
            {
                curr_cell_idx = cell_index_slice[cell_counter];
                ExecuteFloorOperation(curr_cell_idx, Point2D(slice_list[i][j].x, slice_list[i][j].y));
                cell_counter++;
            }
        }
    }
}



int main() {

    map = cv::Mat::zeros(400, 400, CV_32FC3);
    Polygon polygon1, polygon2;
    cv::LineIterator line1(map, cv::Point(200,300), cv::Point(300,200));
    cv::LineIterator line2(map, cv::Point(300,200), cv::Point(200,100));
    cv::LineIterator line3(map, cv::Point(200,100), cv::Point(100,200));
    cv::LineIterator line4(map, cv::Point(100,200), cv::Point(200,300));

    for(int i = 0; i < line1.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line1.pos().x, line1.pos().y));
        line1++;
    }
    for(int i = 0; i < line2.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line2.pos().x, line2.pos().y));
        line2++;
    }
    for(int i = 0; i < line3.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line3.pos().x, line3.pos().y));
        line3++;
    }
    for(int i = 0; i < line4.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line4.pos().x, line4.pos().y));
        line4++;
    }

    cv::LineIterator line5(map, cv::Point(320,350), cv::Point(370,300));
    cv::LineIterator line6(map, cv::Point(370,300), cv::Point(320,250));
    cv::LineIterator line7(map, cv::Point(320,250), cv::Point(270,300));
    cv::LineIterator line8(map, cv::Point(270,300), cv::Point(320,350));
    for(int i = 0; i < line5.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line5.pos().x, line5.pos().y));
        line5++;
    }
    for(int i = 0; i < line6.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line6.pos().x, line6.pos().y));
        line6++;
    }
    for(int i = 0; i < line7.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line7.pos().x, line7.pos().y));
        line7++;
    }
    for(int i = 0; i < line8.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line8.pos().x, line8.pos().y));
        line8++;
    }


    PolygonList polygons = {polygon1, polygon2};
    std::vector<Event> event_list = EventListGenerator(polygons);


    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
    InitializeCellDecomposition(Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition(slice_list);
    FinishCellDecomposition(Point2D(slice_list.back().back().x, slice_list.back().back().y));
    WalkingThroughGraph(0);

    std::cout<<cell_graph.size()<<std::endl;

    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
        {
            std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
        }
    }


    std::vector<cv::Point> contour1 = {cv::Point(200,300), cv::Point(300,200), cv::Point(200,100), cv::Point(100,200)};
    std::vector<cv::Point> contour2 = {cv::Point(320,350), cv::Point(370,300), cv::Point(320,250), cv::Point(270,300)};
    std::vector<std::vector<cv::Point>> contours = {contour1, contour2};
    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));

    for(int i = 0; i < cell_graph.size(); i++)
    {
        drawing_test(cell_graph[i]);
        cv::imshow("cells", map);
        cv::waitKey(0);
    }

    std::vector<Point2D> sub_path;

    for(int i = path.size()-1; i >= 0; i--)
    {
        if(path[i].isCleaned)
        { continue;}
        sub_path = GetBoustrophedonPath(path[i].ceiling, path[i].floor);
        for(int j = 0; j < sub_path.size(); j++)
        {
            map.at<cv::Vec3f>(sub_path[j].y, sub_path[j].x) = cv::Vec3f(0, 255, 0);
            cv::imshow("trajectory", map);
            cv::waitKey(1);
        }
        path[i].isCleaned = true;
    }
    cv::waitKey(0);

    return 0;
}