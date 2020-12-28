#include <iostream>
// #include <climits>
#include <vector>
#include <algorithm>
// #include <string>
// #include <cmath>
#include <numeric>
#include <queue>
#include <list>
#include <ctime>
#include <cmath>

using namespace std;

const double inf = numeric_limits<double>::infinity();

class Label {
public:
    double time, load, cost;
    vector<int> flag, path;
    Label(double time, double load, vector<int>& flag, double cost, vector<int>& path) \
        : time(time), load(load), flag(flag), cost(cost), path(path) {}    
};

class LabelingAlgorithm {
private:
    int n_nodes, origin, destination;
    double capacity, max_T;
    vector<double> early_time, late_time, service_time;
    vector< vector<double> > cost, time, load;
    vector< vector<int> > forward_star;
    vector<int> critical_nodes;
    int new_non_dominated_labels, new_node_added_to_E;

    void graph_reduction();
    double calculate_max_T();
    double calculate_path_time(vector<int>& path);
    double calculate_path_cost(vector<int>& path);
    void update_flag(Label& label);
    bool forward_reachable(Label& label, int v_i, int v_j);
    void forward_search(int v_i, vector<list<Label>>& label_set, queue<int>& set_E);
    bool EFF(vector<list<Label>>& label_set, Label& label, int v_j);
    bool dominate(Label& label1, Label& label2);
    bool is_identical(Label& label1, Label& label2);
    vector<int> find_best_label(vector<list<Label>>& label_set);
    void print_vec(vector<int> &input);
    void print_queue(queue<int> q);


public:
    LabelingAlgorithm(int n, int o, int d, double cap, vector< vector<double> >& c, vector< vector<double> >& t, vector< vector<double> >& l, vector<double>& et, vector<double>& lt, vector<double>& st, vector< vector<int> >& fs);

    vector<int> monodirectional();
};
