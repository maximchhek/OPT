#include <bits/stdc++.h>
using namespace std;

const double MAX_DISTANCE = 20.0;
const double EPS = 1e-9;
const double INF = 1e18;

int n, m, k;
vector<vector<pair<int, double>>> graph;
vector<int> landmarks;
vector<vector<double>> dist_matrix;
vector<int> best_day_order;

struct State {
    double dist;
    int node;
    bool operator>(const State& other) const {
        return dist > other.dist;
    }
};

void dijkstra(int start, vector<double>& dist) {
    fill(dist.begin(), dist.end(), INF);
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
                pq.push({new_dist, v});
            }
        }
    }
}

void build_landmark_distances() {
    vector<double> temp_dist(n + 1);
    
    for (int i = 0; i < k; i++) {
        dijkstra(landmarks[i], temp_dist);
        for (int j = 0; j < k; j++) {
            dist_matrix[i][j] = temp_dist[landmarks[j]];
        }
    }
}

pair<int, vector<pair<int, int>>> solve() {
    if (k == 0) return {0, {}};
    
    build_landmark_distances();
    
    vector<int> dp(1 << k, INT_MAX);
    vector<pair<int, int>> parent(1 << k, {-1, -1});
    
    dp[0] = 0;
    int full_mask = (1 << k) - 1;
    
    for (int mask = 0; mask <= full_mask; mask++) {
        if (dp[mask] == INT_MAX) continue;
        
        for (int start = 0; start < k; start++) {
            if (mask & (1 << start)) continue;
            
            int reachable = 0;
            for (int j = 0; j < k; j++) {
                if (dist_matrix[start][j] <= MAX_DISTANCE + EPS) {
                    reachable |= (1 << j);
                }
            }
            
            for (int submask = reachable; submask > 0; submask = (submask - 1) & reachable) {
                if (submask & (1 << start)) {
                    int new_mask = mask | submask;
                    int new_cost = dp[mask] + 1;
                    if (new_cost < dp[new_mask]) {
                        dp[new_mask] = new_cost;
                        parent[new_mask] = {mask, start};
                    }
                }
            }
        }
    }
    
    vector<pair<int, int>> routes;
    int current_mask = full_mask;
    
    while (current_mask > 0) {
        auto [prev_mask, start_landmark] = parent[current_mask];
        int day_mask = current_mask ^ prev_mask;
        routes.push_back({start_landmark, day_mask});
        current_mask = prev_mask;
    }
    
    reverse(routes.begin(), routes.end());
    return {dp[full_mask], routes};
}

vector<int> reconstruct_path(int start_node, int end_node, vector<double>& dist, vector<int>& par) {
    if (start_node == end_node) return {start_node};
    
    fill(dist.begin(), dist.end(), INF);
    fill(par.begin(), par.end(), -1);
    
    priority_queue<State, vector<State>, greater<State>> pq;
    dist[start_node] = 0;
    pq.push({0, start_node});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (u == end_node) break;
        if (d > dist[u] + EPS) continue;
        
        for (auto& [v, w] : graph[u]) {
            double new_dist = dist[u] + w;
            if (new_dist < dist[v] - EPS) {
                dist[v] = new_dist;
                par[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
    
    vector<int> path;
    int current = end_node;
    while (current != -1) {
        path.push_back(current);
        current = par[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);
    
    cin >> n >> m >> k;
    
    graph.resize(n + 1);
    
    for (int i = 0; i < n; i++) {
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
    
    auto [min_days, routes] = solve();
    
    cout << min_days << "\n";
    
    vector<double> temp_dist(n + 1);
    vector<int> temp_par(n + 1);
    
    for (auto [start_landmark_idx, day_mask] : routes) {
        vector<int> day_indices;
        
        for (int i = 0; i < k; i++) {
            if (day_mask & (1 << i)) {
                day_indices.push_back(i);
            }
        }
        
        sort(day_indices.begin(), day_indices.end(), 
             [start_landmark_idx](int a, int b) {
                 return dist_matrix[start_landmark_idx][a] < dist_matrix[start_landmark_idx][b];
             });
        
        int current_node = landmarks[start_landmark_idx];
        vector<int> path;
        path.push_back(current_node);
        
        for (int idx : day_indices) {
            if (idx == start_landmark_idx) continue;
            
            int next_node = landmarks[idx];
            if (current_node == next_node) continue;
            
            vector<int> segment = reconstruct_path(current_node, next_node, temp_dist, temp_par);
            
            for (size_t i = 1; i < segment.size(); i++) {
                path.push_back(segment[i]);
            }
            
            current_node = next_node;
        }
        
        cout << path.size() << "\n";
        for (size_t i = 0; i < path.size(); i++) {
            if (i > 0) cout << " ";
            cout << path[i];
        }
        cout << "\n";
    }
    
    return 0;
}
