// g++ -std=c++11 -dynamiclib -o liblabeling.dylib labeling.cpp; julia-1.3

#include "labeling.h"

LabelingAlgorithm::LabelingAlgorithm(int n, int o, int d, double cap, vector< vector<double> >& c, vector< vector<double> >& t, vector< vector<double> >& l, vector<double>& et, vector<double>& lt, vector<double>& st, vector< vector<int> >& fs) : n_nodes(n), origin(o), destination(d), capacity(cap), cost(c), time(t), load(l), early_time(et), late_time(lt), service_time(st), forward_star(fs) {

    graph_reduction();
    max_T = calculate_max_T();
    new_non_dominated_labels = 0;
    new_node_added_to_E = 0;
    return;
};

double LabelingAlgorithm::calculate_max_T() {
    double tmp = 0.0;
    double max_tmp = 0.0;
    for (int i=0; i < n_nodes; i++) {
        if (time.at(i).at(destination) < inf) {
            tmp = late_time.at(i) + service_time.at(i) + time.at(i).at(destination);
            max_tmp = max(tmp, max_tmp);
        }
    }
    return min(max_tmp, late_time.at(destination));
}

double LabelingAlgorithm::calculate_path_cost(vector<int>& path) {
    double total_cost = 0.0;
    for (int k=0; k < path.size()-1; k++) {
        int i = path.at(k);
        int j = path.at(k+1);
        total_cost += cost.at(i).at(j);
    }
    return total_cost;
}

void LabelingAlgorithm::graph_reduction() {
    cost.at(origin).at(destination) = inf;

    for (int i=0; i < n_nodes; i++) {
        cost.at(i).at(i) = inf;
        for (int j=0; j < n_nodes; j++) {
            if (i!=j && i!=origin && i!=destination && j!=origin && j!=destination) {
                if (early_time.at(i) + service_time.at(i) + time.at(i).at(j) > late_time.at(j)) {
                    cost.at(i).at(j) = inf;
                }
            }
        }
    }
};

bool LabelingAlgorithm::forward_reachable(Label& label, int v_i, int v_j) {
    // Check time
    double arrival_time = max( label.time + service_time.at(v_i) + time.at(v_i).at(v_j), early_time.at(v_j));
    if (arrival_time > late_time.at(v_j)) return false;

    // check capacity
    double new_load = label.load + load.at(v_i).at(v_j);
    if (new_load > capacity) return false;

    return true;
}

void LabelingAlgorithm::update_flag(Label& label) {
    int v_j = label.path.back();
    label.flag.at(v_j) = 1;

    for (int v_k : forward_star.at(v_j)) {
        if (label.flag.at(v_k)== 0) {
            if (! forward_reachable(label, v_j, v_k) ) {
                label.flag.at(v_k)= 1;
            }
        }
    }
    return;
}

bool LabelingAlgorithm::is_identical(Label& label1, Label& label2) {
    bool is_same = true; 
    if (label1.path.size() != label2.path.size())
        is_same = false;
    else 
        for (int i=0; i<label1.path.size(); i++) { 
            if (label1.path.at(i) != label2.path.at(i))
                is_same = false;
        }

    bool has_same_values = true;
    if (label1.cost != label2.cost) has_same_values = false;
    if (label1.time != label2.time) has_same_values = false;
    if (label1.load != label2.load) has_same_values = false;

    if (is_same) {
        if (!has_same_values)
            cout << "Same Path!!, but different alues!!!\n";
        return true; 
    } else {
        return false;
    }
}

bool LabelingAlgorithm::dominate(Label& label1, Label& label2) {
//    cout << "Comparing label 1 cost " << label1.cost ;
//    cout << " vs. label 2 cost " << label2.cost << endl;
    if (label1.cost > label2.cost)
        return false;
    else if (label1.time > label2.time)
        return false;
    else if (label1.load > label2.load) 
        return false; 
    else
        for (int i=0; i<label1.flag.size(); i++) {
            if (label1.flag.at(i) > label2.flag.at(i)) 
                return false;
        }

    return true;
}

bool LabelingAlgorithm::EFF(vector< list<Label> >& label_set, Label& label, int v_j) {
    bool is_updated = false;

    list<Label>::iterator it;
    for (it = label_set.at(v_j).begin(); it != label_set.at(v_j).end();) {
        if (dominate(label, *it) && !is_identical(label, *it)) {
            it = label_set.at(v_j).erase(it);
            is_updated = true;
        } else {
            ++it;
        }
    }

    bool is_non_dominated = true;
    for (auto& it : label_set.at(v_j)) {
        if (dominate(it, label) || is_identical(label, it)) {
            is_non_dominated = false;
        }
    }    

    
    if (is_non_dominated) {
        label_set.at(v_j).push_back(label);
        is_updated = true;
        new_non_dominated_labels ++;
    }

    return is_updated;
}

void LabelingAlgorithm::forward_search(int v_i, vector< list<Label> >& label_set, queue<int>& set_E) {
    for (Label label : label_set.at(v_i)) {
        for (int v_j : forward_star.at(v_i)) {
            if (label.flag.at(v_j) == 0) { 
                //critical node info should be included later.
                if (forward_reachable(label, v_i, v_j)) {
                    double new_time = max( label.time + service_time.at(v_i) + time.at(v_i).at(v_j), early_time.at(v_j));
                    double new_load = label.load + load.at(v_i).at(v_j);
                    double new_cost = label.cost + cost.at(v_i).at(v_j);
                    vector<int> new_path(label.path);
                    new_path.push_back(v_j);
                    vector<int> new_flag(label.flag);
                    Label new_label(new_time, new_load, new_flag, new_cost, new_path);
                    update_flag(new_label);
                    // print_vec(new_label.flag);

                    bool is_updated = EFF(label_set, new_label, v_j);
                    if (is_updated) {
                        set_E.push(v_j);
                        new_node_added_to_E ++ ;
                    }
                }   
            }
        }
    }
}

vector<int> LabelingAlgorithm::find_best_label(vector< list<Label> >& label_set) {
    double best_cost = inf;
    vector<int> opt_path;
    for (auto const& it : label_set.at(destination)) {
        if (it.cost < best_cost) {
            best_cost = it.cost;
            opt_path = it.path;
        }
    } 
    cout << "best_cost = " << best_cost << endl;
    return opt_path;
}

void LabelingAlgorithm::print_vec(vector<int>& input) {
    cout << "vector { ";
    for (int i = 0; i < input.size(); i++) {
        cout << input.at(i) << ", ";
    }
    cout << "}";    
    cout << endl;
}

void LabelingAlgorithm::print_queue(queue<int> q) {
	//printing content of queue 
    cout << "queue {";
	while (!q.empty()){
		cout<<q.front()<<", ";
		q.pop();
	}
    cout << "}";
	cout<<endl;
}

vector<int> LabelingAlgorithm::monodirectional() {

    cout << "monodirectional()" << endl;
	clock_t time_req;
	time_req = clock();

    critical_nodes = vector<int>(n_nodes-1);
    iota(critical_nodes.begin(), critical_nodes.end(), 0);

    // Initial Label Set
    vector< list<Label> > label_set;
    for (int v_i=0; v_i < n_nodes; v_i++) {
        list<Label> tmp_label;
        label_set.push_back(tmp_label);
    }

    // Label at the origin 
    vector<int> unreachable(n_nodes, 0);
    vector<int> path{origin};
    Label init_label(early_time.at(origin), 0.0, unreachable, 0.0, path);
    update_flag(init_label);
    label_set.at(origin).push_back(init_label);

    // Initial search nodes 
    queue<int> set_E;
    set_E.push(origin);
    while (!set_E.empty()) {
        // select v_i from set_E (not yet deleted)
        int v_i = set_E.front();
        // forward search and extension
        forward_search(v_i, label_set, set_E);
        
        // remove v_i from set_E

        set_E.pop();

    }

    vector<int> opt_path = find_best_label(label_set);

    cout << "new_non_dominated_labels = " << new_non_dominated_labels << endl;
    cout << "new_node_added_to_E = " << new_node_added_to_E << endl;

    print_vec(opt_path);
	time_req = clock() - time_req;

	cout << "Duration: " << (float)time_req/CLOCKS_PER_SEC << " seconds" << endl;

    return opt_path;
    // return opt_path + 1
};

