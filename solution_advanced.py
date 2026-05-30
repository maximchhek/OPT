"""
Расширенная версия с поддержкой больших графов и оптимизациями памяти.
Использует стек вместо рекурсии и кэширование.
"""

import sys
from collections import defaultdict
import heapq
from math import inf

class OptimizedTourPlanner:
    def __init__(self, n, m, k, intersections, graph, landmarks):
        self.n = n
        self.m = m
        self.k = k
        self.intersections = intersections
        self.graph = graph
        self.landmarks = landmarks
        self.MAX_DISTANCE = 20.0
        self.dist_matrix = None
        self.reachable_cache = {}
        
    def dijkstra(self, start):
        """Dijkstra с поддержкой больших графов"""
        dist = [inf] * (self.n + 1)
        parent = [-1] * (self.n + 1)
        dist[start] = 0
        pq = [(0, start)]
        
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v, w in self.graph[u]:
                new_dist = dist[u] + w
                if new_dist < dist[v]:
                    dist[v] = new_dist
                    parent[v] = u
                    heapq.heappush(pq, (new_dist, v))
        
        return dist, parent
    
    def build_landmark_distances(self):
        """Строит матрицу расстояний между достопримечательностями"""
        self.dist_matrix = [[inf] * self.k for _ in range(self.k)]
        
        for i in range(self.k):
            dist, _ = self.dijkstra(self.landmarks[i])
            for j in range(self.k):
                self.dist_matrix[i][j] = dist[self.landmarks[j]]
    
    def get_reachable_set(self, start_idx):
        """Получает маску достижимых достопримечательностей (с кэшем)"""
        if start_idx in self.reachable_cache:
            return self.reachable_cache[start_idx]
        
        reachable = 0
        for j in range(self.k):
            if self.dist_matrix[start_idx][j] <= self.MAX_DISTANCE:
                reachable |= (1 << j)
        
        self.reachable_cache[start_idx] = reachable
        return reachable
    
    def solve_with_early_termination(self):
        """Решает с ранней остановкой и оптимизациями"""
        if self.k == 0:
            return 0, []
        
        self.build_landmark_distances()
        
        # DP с ограничением памяти
        dp = [inf] * (1 << self.k)
        dp[0] = 0
        parent = [(-1, -1) for _ in range(1 << self.k)]
        
        full_mask = (1 << self.k) - 1
        
        for mask in range(1 << self.k):
            if dp[mask] == inf:
                continue
            
            if mask == full_mask:
                break
            
            # Пробуем все непосещённые достопримечательности как начало дня
            for start in range(self.k):
                if mask & (1 << start):
                    continue
                
                reachable = self.get_reachable_set(start)
                
                # Итеративный перебор подмножеств
                submask = reachable
                while submask > 0:
                    if submask & (1 << start):
                        new_mask = mask | submask
                        new_cost = dp[mask] + 1
                        
                        if new_cost < dp[new_mask]:
                            dp[new_mask] = new_cost
                            parent[new_mask] = (mask, start)
                    
                    submask = (submask - 1) & reachable
        
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
    
    def reconstruct_path_iterative(self, start_node, end_node):
        """Восстанавливает путь итеративно (без рекурсии)"""
        dist, parent = self.dijkstra(start_node)
        
        path = []
        current = end_node
        
        while current != -1:
            path.append(current)
            current = parent[current]
        
        path.reverse()
        return path
    
    def build_daily_routes(self, routes):
        """Строит полные маршруты с оптимизацией"""
        daily_routes = []
        
        for start_landmark_idx, day_mask in routes:
            day_indices = []
            for i in range(self.k):
                if day_mask & (1 << i):
                    day_indices.append(i)
            
            # Сортируем по расстоянию от начальной точки
            day_indices.sort(key=lambda i: self.dist_matrix[start_landmark_idx][i])
            
            # Строим маршрут
            current_node = self.landmarks[start_landmark_idx]
            path = [current_node]
            
            for next_idx in day_indices[1:]:
                next_node = self.landmarks[next_idx]
                segment = self.reconstruct_path_iterative(path[-1], next_node)
                path.extend(segment[1:])
            
            daily_routes.append(path)
        
        return daily_routes
    
    def validate_solution(self, daily_routes):
        """Проверяет корректность решения"""
        visited_landmarks = set()
        total_distance = 0
        
        for route in daily_routes:
            day_distance = 0
            for i in range(len(route) - 1):
                # Находим расстояние между соседними вершинами
                u, v = route[i], route[i + 1]
                found = False
                for neighbor, dist in self.graph[u]:
                    if neighbor == v:
                        day_distance += dist
                        found = True
                        break
                if not found:
                    return False, f"Нет ребра между {u} и {v}"
            
            if day_distance > self.MAX_DISTANCE * 1000 + 1:  # +1 для погрешности
                return False, f"День превышает лимит: {day_distance}км"
            
            for node in route:
                if node in self.landmarks:
                    visited_landmarks.add(self.landmarks.index(node))
            
            total_distance += day_distance
        
        if len(visited_landmarks) != len(self.landmarks):
            return False, f"Не все достопримечательности посещены: {visited_landmarks}"
        
        return True, "OK"

def main():
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
    
    planner = OptimizedTourPlanner(n, m, k, intersections, graph, landmarks)
    min_days, routes = planner.solve_with_early_termination()
    
    if min_days == 0:
        print(0)
        return
    
    daily_routes = planner.build_daily_routes(routes)
    
    # Валидация
    valid, msg = planner.validate_solution(daily_routes)
    if not valid and len(sys.argv) > 1:
        print(f"Ошибка: {msg}", file=sys.stderr)
    
    print(min_days)
    for route in daily_routes:
        print(len(route))
        print(' '.join(map(str, route)))

if __name__ == "__main__":
    main()
