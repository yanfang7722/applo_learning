
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

const bool PROCESS_PLOT = true;

double xmin = -10.0;
double ymin = -10.0;
double xmax = 60.0;
double ymax = 60.0;
double grid_size = 2.0;
double robot_size = 2.0;

string file_dir = "//media/yangf/document/test_c++/Dijkstra/obs_map.csv";

void LoadEnvFile(string file_dir, vector<array<double, 2>>& env) {
  ifstream csv_input;
  csv_input.open(file_dir);
  if (!csv_input.is_open()) {
    cout << " open file failed !!" << endl;
    exit(0);
  } else {
    while (!csv_input.eof()) {
      array<double, 2> obs_coor;
      csv_input >> obs_coor[0];
      csv_input.ignore(255, ',');
      csv_input >> obs_coor[1];
      csv_input.ignore(255, '\n');
      env.push_back(obs_coor);
    }
  }
}

void SaveResult(vector<vector<double>>& paths) {
  ofstream outFile;
  string result_dir = file_dir.insert(file_dir.length() - 4, "_result");
  outFile.open(result_dir, ios::out);
  outFile << "x"
          << ","
          << "y" << endl;
  for (int i = paths[0].size() - 1; i >= 0; i--) {
    outFile << paths[0][i] << ", " << paths[1][i] << std::endl;
  }
  outFile.close();
}

class Node {
 public:
  int i;
  int j;
  int st;  // 0: default,1:open list,2:close list,3:unreachable
  double cost;
  double x;
  double y;
  Node* p_node;

  void xyTOij() {
    i = (int)(x - xmin) / grid_size;
    j = (int)(y - ymin) / grid_size;
  }
  void ijTOxy() {
    x = i * grid_size + xmin;
    y = j * grid_size + ymin;
  }
  Node(double x_ = 0, double y_ = 0, double cost_ = 0, Node* p_node_ = NULL)
      : x(x_), y(y_), cost(cost_), p_node(p_node_) {
    xyTOij();
    st = 0;
  };
  Node(int i_ = 0, int j_ = 0, double cost_ = 0, Node* p_node_ = NULL)
      : i(i_), j(j_), cost(cost_), p_node(p_node_) {
    ijTOxy();
    st = 0;
  };
  bool operator==(const Node a) {
    if (i == a.i && j == a.j && st == a.st && cost == a.cost)
      return true;
    else
      return false;
  }
};
struct CompareNode {
  bool operator()(Node a, Node b) {
    return a.cost > b.cost;  // xiao
  }
};

vector<vector<double>> calc_final_path(Node* goal) {
  vector<double> rx, ry;
  Node* node = goal;
  do {
    rx.push_back(node->x);
    ry.push_back(node->y);
    node = node->p_node;
  } while (node != NULL);
  return {rx, ry};
}

void CalcObstacleMap(vector<array<double, 2>> obs,
                     vector<vector<Node>>& obmap) {
  int xwidth = (int)(xmax - xmin) / grid_size + 1;
  int ywidth = (int)(ymax - ymin) / grid_size + 1;
  for (int i = 0; i < xwidth; i++) {
    vector<Node> col;
    for (int j = 0; j < ywidth; j++) {
      Node a(i, j);
      for (int k = 0; k < obs.size(); k++) {
        double d = sqrt(pow((obs[k][0] - a.x), 2) + pow((obs[k][1] - a.y), 2));
        if (d <= robot_size) {
          a.st = 3;
          break;
        }
      }
      col.push_back(a);
    }
    obmap.push_back(col);
  }
}

void AddNeighbor(Node& st, vector<vector<Node>>& map,
                 priority_queue<Node, vector<Node>, CompareNode>& open_list) {
  vector<Node> neighbor{Node(st.i + 1, st.j, st.cost + 1, &st),
                        Node(st.i, st.j + 1, st.cost + 1, &st),
                        Node(st.i - 1, st.j, st.cost + 1, &st),
                        Node(st.i, st.j - 1, st.cost + 1, &st)};
  for (auto iter = neighbor.begin(); iter != neighbor.end(); iter++) {
    if (st.p_node && st.p_node->i != iter->i && st.p_node->j != iter->j) {
      iter->cost += 0.2;
    }
    if (iter->i >= 0 && iter->i < map.size() && iter->j >= 0 &&
        iter->j < map[0].size()) {
      iter->st = 1;
      if (map[iter->i][iter->j].st == 0) {
        map[iter->i][iter->j] = *iter;
        open_list.push(*iter);
      } else if (map[iter->i][iter->j].st == 1 &&
                 map[iter->i][iter->j].cost > iter->cost) {
        map[iter->i][iter->j] = *iter;
        open_list.push(*iter);
      }
    }
  }
}

void PlotMap(vector<vector<Node>>& map, Node& st, Node& end, Node& active) {
  plt::clf();
  plt::plot({st.x}, {st.y}, "ro");
  plt::plot({end.x}, {end.y}, "rx");
  array<vector<double>, 3> x, y;
  vector<vector<double>> path;
  for (int i = 0; i < map.size(); i++) {
    for (int j = 0; j < map[0].size(); j++) {
      if (map[i][j].st) {
        x[map[i][j].st - 1].push_back(map[i][j].x);
        y[map[i][j].st - 1].push_back(map[i][j].y);
      }
    }
  }
  plt::plot(x[0], y[0], "g*");
  plt::plot(x[2], y[2], "ks");
  plt::scatter(x[1], y[1]);
  path = calc_final_path(&map[active.i][active.j]);
  plt::plot(path[0], path[1], "r");
  plt::show(false);
  plt::pause(0.001);
  if (active == end) {
    plt::save("result.png");
  }
}

void dijkstra_star_planning(Node& st, Node& end, vector<vector<Node>>& map) {
  priority_queue<Node, vector<Node>, CompareNode> open_list;
  open_list.push(st);
  // stop when search all the map or find the path
  while ((!open_list.empty()) && map[end.i][end.j].st != 2) {
    // find active
    Node active = open_list.top();
    while (map[active.i][active.j].st == 2) {  // delete duplicate
      open_list.pop();
      active = open_list.top();
    }
    open_list.pop();
    map[active.i][active.j].st = 2;
    // update open list
    AddNeighbor(map[active.i][active.j], map, open_list);
    if (PROCESS_PLOT) PlotMap(map, st, end, active);
  }
};

int main() {
  double sx = 0.0;
  double sy = 0.0;
  double gx = 50.0;
  double gy = 30.0;
  cout << "main start" << endl;
  vector<array<double, 2>> obsdata;
  LoadEnvFile(file_dir, obsdata);
  vector<vector<Node>> map;
  CalcObstacleMap(obsdata, map);

  Node st(sx, sy, 0);
  Node end(gx, gy, 0);
  PlotMap(map, st, map[end.i][end.j], st);
  dijkstra_star_planning(st, end, map);

  PlotMap(map, st, map[end.i][end.j], map[end.i][end.j]);
  auto path = calc_final_path(&map[end.i][end.j]);
  SaveResult(path);
  cout << "main finish" << endl;
  return 0;
}