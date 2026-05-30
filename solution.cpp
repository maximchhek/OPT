#include <bits/stdc++.h>
using namespace std;

const double MAX_DISTANCE = 20.0;
const double INF = 1e18;

int n, m, k;
vector<pair<double, double>> intersections;
vector<vector<pair<int, double>>> graph;
vector<int> landmarks;
vector<vector<double>> dist_matrix;

vector<double> dijkstra(int start) {
    vector<double> dist(n + 1, INF);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    
    dist[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        
        for (auto [v, w] : graph[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    
    return dist;
}

void build_landmark_distances() {
    dist_matrix.assign(k, vector<double>(k));
    
    for (int i = 0; i < k; i++) {
        vector<double> dist = dijkstra(landmarks[i]);
        for (int j = 0; j < k; j++) {
            dist_matrix[i][j] = dist[landmarks[j]];
        }
    }
}

int get_reachable_set(int start_idx) {
    int reachable = 0;
    for (int j = 0; j < k; j++) {
        if (dist_matrix[start_idx][j] <= MAX_DISTANCE) {
            reachable |= (1 << j);
        }
    }
    return reachable;
}

pair<int, vector<pair<int, int>>> solve() {
    if (k == 0) return {0, {}};
    
    build_landmark_distances();
    
    vector<double> dp(1 << k, INF);
    vector<pair<int, int>> parent(1 << k, {-1, -1});
    
    dp[0] = 0;
    
    int full_mask = (1 << k) - 1;
    
    for (int mask = 0; mask < (1 << k); mask++) {
        if (dp[mask] == INF) continue;
        
        for (int start = 0; start < k; start++) {
            if (mask & (1 << start)) continue;
            
            int reachable = get_reachable_set(start);
            
            for (int submask = reachable; submask > 0; submask = (submask - 1) & reachable) {
                if (submask & (1 << start)) {
                    int new_mask = mask | submask;
                    if (dp[mask] + 1 < dp[new_mask]) {
                        dp[new_mask] = dp[mask] + 1;
                        parent[new_mask] = {mask, start};
                    }
                }
            }
        }
    }
    
    int min_days = (int)dp[full_mask];
    
    vector<pair<int, int>> routes;
    int current_mask = full_mask;
    
    while (current_mask > 0) {
        int prev_mask = parent[current_mask].first;
        int start_landmark = parent[current_mask].second;
        int day_mask = current_mask ^ prev_mask;
        routes.push_back({start_landmark, day_mask});
        current_mask = prev_mask;
    }
    
    reverse(routes.begin(), routes.end());
    
    return {min_days, routes};
}

vector<int> reconstruct_path(int start_node, int end_node) {
    vector<double> dist(n + 1, INF);
    vector<int> par(n + 1, -1);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    
    dist[start_node] = 0;
    pq.push({0, start_node});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        if (u == end_node) break;
        
        for (auto [v, w] : graph[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                par[v] = u;
                pq.push({dist[v], v});
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

vector<vector<int>> build_daily_routes(const vector<pair<int, int>>& routes) {
    vector<vector<int>> daily_routes;
    
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
        vector<int> path = {current_node};
        
        for (int idx : day_indices) {
            if (idx == start_landmark_idx) continue;
            int next_node = landmarks[idx];
            vector<int> segment = reconstruct_path(path.back(), next_node);
            for (int i = 1; i < (int)segment.size(); i++) {
                path.push_back(segment[i]);
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
    
    intersections.resize(n + 1);
    for (int i = 1; i <= n; i++) {
        double lat, lon;
        cin >> lat >> lon;
        intersections[i] = {lat, lon};
    }
    
    graph.resize(n + 1);
    for (int i = 0; i < m; i++) {
        int u, v;
        double length;
        cin >> u >> v >> length;
        length = length / 1000.0;
        graph[u].push_back({v, length});
        graph[v].push_back({u, length});
    }
    
    landmarks.resize(k);
    for (int i = 0; i < k; i++) {
        cin >> landmarks[i];
    }
    
    auto [min_days, routes] = solve();
    
    if (min_days == 0) {
        cout << 0 << "\n";
        return 0;
    }
    
    vector<vector<int>> daily_routes = build_daily_routes(routes);
    
    cout << min_days << "\n";
    for (const auto& route : daily_routes) {
        cout << route.size() << "\n";
        for (int i = 0; i < (int)route.size(); i++) {
            if (i > 0) cout << " ";
            cout << route[i];
        }
        cout << "\n";
    }
    
    return 0;
}
