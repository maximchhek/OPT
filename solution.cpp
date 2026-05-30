#include <bits/stdc++.h>
using namespace std;

const double MAX_DISTANCE = 20.0;
const double EPS = 1e-9;
const double INF = 1e18;

int n, m, k;
vector<vector<pair<int, double>>> graph;
vector<int> landmarks;
vector<vector<double>> dist_matrix;

struct State {
    double dist;
    int node;
    bool operator>(const State& other) const {
        return dist > other.dist;
    }
};

void dijkstra(int start, vector<double>& dist, vector<int>& parent) {
    fill(dist.begin(), dist.end(), INF);
    fill(parent.begin(), parent.end(), -1);
    
    priority_queue<State, vector<State>, greater<State>> pq;
    dist[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u] + EPS) continue;
        
        for (auto& [v, w] : graph[u]) {
            double new_dist = dist[u] + w;
            if (new_dist < dist[v] - EPS) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
}

void build_landmark_distances() {
    vector<double> temp_dist(n + 1);
    vector<int> temp_parent(n + 1);
    
    for (int i = 0; i < k; i++) {
        dijkstra(landmarks[i], temp_dist, temp_parent);
        for (int j = 0; j < k; j++) {
            dist_matrix[i][j] = temp_dist[landmarks[j]];
        }
    }
}

vector<int> reconstruct_path(int start_node, int end_node, const vector<int>& parent) {
    vector<int> path;
    int current = end_node;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<vector<int>> solve_greedy() {
    if (k == 0) return {};
    
    build_landmark_distances();
    
    vector<bool> visited(k, false);
    vector<vector<int>> daily_routes;
    
    while (true) {
        bool all_visited = true;
        for (int i = 0; i < k; i++) {
            if (!visited[i]) {
                all_visited = false;
                break;
            }
        }
        if (all_visited) break;
        
        int best_start = -1;
        vector<int> best_day_landmarks;
        int best_day_cost = INT_MAX;
        
        for (int start = 0; start < k; start++) {
            if (visited[start]) continue;
            
            vector<pair<double, int>> candidates;
            for (int j = 0; j < k; j++) {
                if (!visited[j]) {
                    candidates.push_back({dist_matrix[start][j], j});
                }
            }
            sort(candidates.begin(), candidates.end());
            
            vector<int> day_landmarks;
            double current_dist = 0;
            int current_idx = start;
            day_landmarks.push_back(start);
            
            for (auto [d, idx] : candidates) {
                if (idx == start) continue;
                if (current_dist + dist_matrix[current_idx][idx] <= MAX_DISTANCE + EPS) {
                    day_landmarks.push_back(idx);
                    current_dist += dist_matrix[current_idx][idx];
                    current_idx = idx;
                }
            }
            
            if ((int)day_landmarks.size() < (int)best_day_landmarks.size() || 
                ((int)day_landmarks.size() == (int)best_day_landmarks.size() && best_start == -1)) {
                if ((int)day_landmarks.size() > (int)best_day_landmarks.size()) {
                    best_start = start;
                    best_day_landmarks = day_landmarks;
                } else if (best_start == -1) {
                    best_start = start;
                    best_day_landmarks = day_landmarks;
                }
            }
        }
        
        if (best_start == -1) {
            for (int i = 0; i < k; i++) {
                if (!visited[i]) {
                    best_start = i;
                    best_day_landmarks.push_back(i);
                    break;
                }
            }
        }
        
        for (int idx : best_day_landmarks) {
            visited[idx] = true;
        }
        
        sort(best_day_landmarks.begin() + 1, best_day_landmarks.end(),
             [best_start](int a, int b) {
                 return dist_matrix[best_start][a] < dist_matrix[best_start][b];
             });
        
        vector<int> path;
        path.push_back(landmarks[best_day_landmarks[0]]);
        
        vector<double> temp_dist(n + 1);
        vector<int> temp_parent(n + 1);
        
        for (size_t i = 1; i < best_day_landmarks.size(); i++) {
            int prev_idx = best_day_landmarks[i - 1];
            int next_idx = best_day_landmarks[i];
            
            dijkstra(landmarks[prev_idx], temp_dist, temp_parent);
            vector<int> segment = reconstruct_path(landmarks[next_idx], landmarks[next_idx], temp_parent);
            
            if (segment.empty() || segment[0] != landmarks[next_idx]) {
                segment = {landmarks[next_idx]};
            }
            
            if (path.back() != segment[0]) {
                dijkstra(path.back(), temp_dist, temp_parent);
                segment = reconstruct_path(landmarks[next_idx], landmarks[next_idx], temp_parent);
                if (segment.empty()) segment = {landmarks[next_idx]};
            }
            
            for (size_t j = (path.back() == segment[0] ? 1 : 0); j < segment.size(); j++) {
                path.push_back(segment[j]);
            }
        }
        
        daily_routes.push_back(path);
    }
    
    return daily_routes;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);
    
    cin >> n >> m >> k;
    
    graph.resize(n + 1);
    
    for (int i = 1; i <= n; i++) {
        double lat, lon;
        cin >> lat >> lon;
    }
    
    for (int i = 0; i < m; i++) {
        int u, v;
        double length;
        cin >> u >> v >> length;
        length /= 1000.0;
        graph[u].push_back({v, length});
        graph[v].push_back({u, length});
    }
    
    landmarks.resize(k);
    for (int i = 0; i < k; i++) {
        cin >> landmarks[i];
    }
    
    if (k == 0) {
        cout << 0 << "\n";
        return 0;
    }
    
    dist_matrix.assign(k, vector<double>(k, 0));
    
    vector<vector<int>> daily_routes = solve_greedy();
    
    cout << daily_routes.size() << "\n";
    for (const auto& route : daily_routes) {
        cout << route.size() << "\n";
        for (size_t i = 0; i < route.size(); i++) {
            if (i > 0) cout << " ";
            cout << route[i];
        }
        cout << "\n";
    }
    
    return 0;
}
