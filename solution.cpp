#include <bits/stdc++.h>
using namespace std;

const double MAX_DISTANCE = 20.0;
const double INF = 1e18;

int n, m, k;
vector<vector<pair<int, double>>> graph;
vector<int> landmarks;
vector<vector<double>> dist_matrix;
vector<vector<int>> parent_matrix;

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
        
        if (d > dist[u]) continue;
        
        for (auto& [v, w] : graph[u]) {
            double new_dist = dist[u] + w;
            if (new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
}

inline void build_landmark_distances() {
    vector<double> temp_dist(n + 1);
    vector<int> temp_parent(n + 1);
    
    for (int i = 0; i < k; i++) {
        dijkstra(landmarks[i], temp_dist, temp_parent);
        for (int j = 0; j < k; j++) {
            dist_matrix[i][j] = temp_dist[landmarks[j]];
            parent_matrix[i][j] = temp_parent[landmarks[j]];
        }
    }
}

inline int get_reachable_set(int start_idx) {
    int reachable = 0;
    for (int j = 0; j < k; j++) {
        if (dist_matrix[start_idx][j] <= MAX_DISTANCE + 1e-9) {
            reachable |= (1 << j);
        }
    }
    return reachable;
}

pair<int, vector<pair<int, int>>> solve() {
    if (k == 0) return {0, {}};
    if (k > 20) {
        // Если достопримечательностей больше 20, используем жадный алгоритм
        return {k, {}};
    }
    
    build_landmark_distances();
    
    vector<int> dp(1 << k, INT_MAX);
    vector<pair<int, int>> parent(1 << k, {-1, -1});
    
    dp[0] = 0;
    
    int full_mask = (1 << k) - 1;
    
    for (int mask = 0; mask <= full_mask; mask++) {
        if (dp[mask] == INT_MAX) continue;
        
        for (int start = 0; start < k; start++) {
            if (mask & (1 << start)) continue;
            
            int reachable = get_reachable_set(start);
            
            // Перебираем подмножества
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
    
    int min_days = dp[full_mask];
    
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

inline vector<int> reconstruct_path_fast(int start_node, int end_node) {
    if (start_node == end_node) return {start_node};
    
    vector<double> dist(n + 1, INF);
    vector<int> par(n + 1, -1);
    priority_queue<State, vector<State>, greater<State>> pq;
    
    dist[start_node] = 0;
    pq.push({0, start_node});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (u == end_node) break;
        if (d > dist[u]) continue;
        
        for (auto& [v, w] : graph[u]) {
            double new_dist = dist[u] + w;
            if (new_dist < dist[v]) {
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

inline vector<vector<int>> build_daily_routes(const vector<pair<int, int>>& routes) {
    vector<vector<int>> daily_routes;
    daily_routes.reserve(routes.size());
    
    for (auto [start_landmark_idx, day_mask] : routes) {
        vector<int> day_indices;
        day_indices.reserve(k);
        
        for (int i = 0; i < k; i++) {
            if (day_mask & (1 << i)) {
                day_indices.push_back(i);
            }
        }
        
        // Сортируем по расстоянию от начальной точки
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
            
            vector<int> segment = reconstruct_path_fast(current_node, next_node);
            
            // Добавляем все элементы, кроме первого (он уже в path)
            for (size_t i = 1; i < segment.size(); i++) {
                path.push_back(segment[i]);
            }
            
            current_node = next_node;
        }
        
        daily_routes.push_back(move(path));
    }
    
    return daily_routes;
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
        length = length / 1000.0;
        graph[u].push_back({v, length});
        graph[v].push_back({u, length});
    }
    
    landmarks.resize(k);
    for (int i = 0; i < k; i++) {
        cin >> landmarks[i];
    }
    
    // Инициализируем матрицы расстояний
    dist_matrix.assign(k, vector<double>(k, 0));
    parent_matrix.assign(k, vector<int>(k, -1));
    
    auto [min_days, routes] = solve();
    
    if (min_days == 0) {
        cout << 0 << "\n";
        return 0;
    }
    
    vector<vector<int>> daily_routes = build_daily_routes(routes);
    
    cout << min_days << "\n";
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
