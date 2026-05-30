"""
Оптимизированное решение для задачи планирования туров.
Использует Dynamic Programming с битовыми масками и алгоритм Dijkstra.
"""

import sys
from collections import defaultdict
import heapq
from math import inf

class TourPlanner:
    def __init__(self, n, m, k, intersections, graph, landmarks):
        self.n = n
        self.m = m
        self.k = k
        self.intersections = intersections
        self.graph = graph
        self.landmarks = landmarks
        self.MAX_DISTANCE = 20.0
        self.dist_matrix = None
        self.parent_matrix = None
        
    def dijkstra(self, start):
        """Dijkstra для поиска кратчайших путей от start"""
        dist = [inf] * (self.n + 1)
        parent = [-1] * (self.n + 1)
        dist[start] = 0
        pq = [(0, start)]
        
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v, w in self.graph[u]:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    parent[v] = u
                    heapq.heappush(pq, (dist[v], v))
        
        return dist, parent
    
    def build_landmark_distances(self):
        """Строит матрицы расстояний и родителей между достопримечательностями"""
        self.dist_matrix = [[inf] * self.k for _ in range(self.k)]
        self.parent_matrix = [[-1] * self.k for _ in range(self.k)]
        
        for i in range(self.k):
            dist, parent = self.dijkstra(self.landmarks[i])
            for j in range(self.k):
                self.dist_matrix[i][j] = dist[self.landmarks[j]]
                # Сохраняем parent для восстановления пути
                self.parent_matrix[i][j] = parent[self.landmarks[j]]
    
    def get_reachable_set(self, start_idx):
        """Получает битовую маску достижимых достопримечательностей из start_idx"""
        reachable = 0
        for j in range(self.k):
            if self.dist_matrix[start_idx][j] <= self.MAX_DISTANCE:
                reachable |= (1 << j)
        return reachable
    
    def solve(self):
        """Решает задачу с DP и возвращает минимальное число дней и маршруты"""
        if self.k == 0:
            return 0, []
        
        self.build_landmark_distances()
        
        # DP: dp[mask] = (min_days, start_landmark)
        dp = [inf] * (1 << self.k)
        dp[0] = 0
        parent = [(-1, -1) for _ in range(1 << self.k)]
        
        for mask in range(1 << self.k):
            if dp[mask] == inf:
                continue
            
            # Пробуем начать новый день с каждой непосещённой достопримечательности
            for start in range(self.k):
                if mask & (1 << start):
                    continue
                
                # Получаем маску достижимых достопримечательностей
                reachable = self.get_reachable_set(start)
                
                # Перебираем все подмножества достижимых
                submask = reachable
                while submask > 0:
                    if submask & (1 << start):  # start должен быть в подмножестве
                        new_mask = mask | submask
                        if dp[mask] + 1 < dp[new_mask]:
                            dp[new_mask] = dp[mask] + 1
                            parent[new_mask] = (mask, start)
                    submask = (submask - 1) & reachable
        
        full_mask = (1 << self.k) - 1
        min_days = int(dp[full_mask])
        
        # Восстанавливаем маршруты
        routes = []
        current_mask = full_mask
        while current_mask > 0:
            prev_mask, start_landmark = parent[current_mask]
            day_mask = current_mask ^ prev_mask
            routes.append((start_landmark, day_mask))
            current_mask = prev_mask
        
        routes.reverse()
        return min_days, routes
    
    def reconstruct_path(self, start_node, end_node):
        """Восстанавливает полный путь между двумя вершинами"""
        dist, parent = self.dijkstra(start_node)
        
        path = []
        current = end_node
        while current != -1:
            path.append(current)
            current = parent[current]
        path.reverse()
        
        return path
    
    def build_daily_routes(self, routes):
        """Строит полные маршруты на каждый день с восстановлением путей"""
        daily_routes = []
        
        for start_landmark_idx, day_mask in routes:
            # Собираем индексы достопримечательностей в этом дне
            day_indices = []
            for i in range(self.k):
                if day_mask & (1 << i):
                    day_indices.append(i)
            
            # Сортируем по расстоянию от начальной точки
            day_indices.sort(key=lambda i: self.dist_matrix[start_landmark_idx][i])
            
            # Строим маршрут
            current_idx = start_landmark_idx
            path = [self.landmarks[current_idx]]
            
            for next_idx in day_indices[1:]:  # Пропускаем первый (он уже в пути)
                next_node = self.landmarks[next_idx]
                segment = self.reconstruct_path(path[-1], next_node)
                path.extend(segment[1:])
                current_idx = next_idx
            
            daily_routes.append(path)
        
        return daily_routes

def main():
    # Читаем входные данные
    n, m, k = map(int, input().split())
    
    intersections = [None]
    for i in range(n):
        lat, lon = map(float, input().split())
        intersections.append((lat, lon))
    
    graph = defaultdict(list)
    for _ in range(m):
        u, v, length = map(int, input().split())
        length_km = length / 1000.0
        graph[u].append((v, length_km))
        graph[v].append((u, length_km))
    
    landmarks = []
    for _ in range(k):
        intersection = int(input())
        landmarks.append(intersection)
    
    # Решаем
    planner = TourPlanner(n, m, k, intersections, graph, landmarks)
    min_days, routes = planner.solve()
    
    if min_days == 0:
        print(0)
        return
    
    daily_routes = planner.build_daily_routes(routes)
    
    # Выводим результаты
    print(min_days)
    for route in daily_routes:
        print(len(route))
        print(' '.join(map(str, route)))

if __name__ == "__main__":
    main()
