import sys
from collections import defaultdict, deque
import heapq
from math import inf

def read_input():
    """Читает входные данные"""
    n, m, k = map(int, input().split())
    
    # Читаем координаты перекрестков
    intersections = [None]  # 1-indexed
    for i in range(n):
        lat, lon = map(float, input().split())
        intersections.append((lat, lon))
    
    # Читаем улицы (рёбра графа)
    graph = defaultdict(list)
    for _ in range(m):
        u, v, length = map(int, input().split())
        length_km = length / 1000.0
        graph[u].append((v, length_km))
        graph[v].append((u, length_km))
    
    # Читаем достопримечательности
    landmarks = []
    for _ in range(k):
        intersection = int(input())
        landmarks.append(intersection)
    
    return n, m, k, intersections, graph, landmarks

def dijkstra(graph, start, n):
    """Алгоритм Dijkstra для поиска кратчайших путей от start до всех вершин"""
    dist = [inf] * (n + 1)
    dist[start] = 0
    pq = [(0, start)]
    
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in graph[u]:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                heapq.heappush(pq, (dist[v], v))
    
    return dist

def build_landmark_distances(graph, landmarks, n):
    """Строит матрицу расстояний между достопримечательностями"""
    k = len(landmarks)
    dist_matrix = [[inf] * k for _ in range(k)]
    
    for i in range(k):
        distances = dijkstra(graph, landmarks[i], n)
        for j in range(k):
            dist_matrix[i][j] = distances[landmarks[j]]
    
    return dist_matrix

def find_reachable_landmarks(start_landmark, dist_matrix, k, max_distance=20):
    """Находит достопримечательности, достижимые из start_landmark за день (макс 20км)"""
    reachable = set()
    for j in range(k):
        if dist_matrix[start_landmark][j] <= max_distance:
            reachable.add(j)
    return reachable

def solve_with_dp(landmarks, dist_matrix, k, max_distance=20):
    """
    Решает задачу с использованием Dynamic Programming с битовыми масками.
    Находит минимальное число дней для посещения всех достопримечательностей.
    """
    MAX_DAYS = 20
    
    # dp[mask] = минимальное число дней для посещения достопримечательностей в mask
    dp = [inf] * (1 << k)
    dp[0] = 0
    parent = [(-1, -1) for _ in range(1 << k)]
    
    for mask in range(1 << k):
        if dp[mask] == inf:
            continue
        
        # Пробуем начать новый день с каждой непосещённой достопримечательности
        for start_landmark in range(k):
            if mask & (1 << start_landmark):
                continue  # Уже посещена
            
            # Находим все достопримечательности, которые можно посетить за один день
            reachable = find_reachable_landmarks(start_landmark, dist_matrix, k, max_distance)
            
            # Перебираем все подмножества достижимых достопримечательностей
            submask = reachable
            while submask > 0:
                # Проверяем, что start_landmark входит в submask
                if submask & (1 << start_landmark):
                    new_mask = mask | submask
                    if dp[mask] + 1 < dp[new_mask]:
                        dp[new_mask] = dp[mask] + 1
                        parent[new_mask] = (mask, start_landmark)
                submask = (submask - 1) & reachable
    
    full_mask = (1 << k) - 1
    min_days = dp[full_mask]
    
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

def reconstruct_path(graph, start, end, n):
    """Восстанавливает кратчайший путь между двумя вершинами"""
    dist = [inf] * (n + 1)
    parent = [-1] * (n + 1)
    dist[start] = 0
    pq = [(0, start)]
    
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u == end:
            break
        for v, w in graph[u]:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                parent[v] = u
                heapq.heappush(pq, (dist[v], v))
    
    # Восстанавливаем путь
    path = []
    current = end
    while current != -1:
        path.append(current)
        current = parent[current]
    path.reverse()
    
    return path

def build_daily_routes(graph, landmarks, dist_matrix, n, routes):
    """Строит полные маршруты на каждый день"""
    daily_routes = []
    
    for start_landmark, day_mask in routes:
        # Находим все достопримечательности в этом дне
        day_landmarks = []
        for i in range(len(landmarks)):
            if day_mask & (1 << i):
                day_landmarks.append(i)
        
        # Начинаем с start_landmark
        day_landmarks.sort(key=lambda i: dist_matrix[start_landmark][i])
        
        # Строим маршрут: посещаем достопримечательности в порядке близости
        current = landmarks[start_landmark]
        path = [current]
        visited_landmarks = {start_landmark}
        total_distance = 0
        
        while len(visited_landmarks) < len(day_landmarks):
            best_next = -1
            best_dist = inf
            
            for i in day_landmarks:
                if i not in visited_landmarks:
                    if dist_matrix[day_landmarks.index(current) if current in [landmarks[j] for j in day_landmarks] else start_landmark][i] < best_dist:
                        best_next = i
                        best_dist = dist_matrix[start_landmark if not path else day_landmarks.index(current)][i]
            
            if best_next != -1:
                next_node = landmarks[best_next]
                segment_path = reconstruct_path(graph, current, next_node, n)
                path.extend(segment_path[1:])
                current = next_node
                visited_landmarks.add(best_next)
        
        daily_routes.append(path)
    
    return daily_routes

def solve():
    """Основная функция решения"""
    n, m, k, intersections, graph, landmarks = read_input()
    
    if k == 0:
        print(0)
        return
    
    # Строим матрицу расстояний между достопримечательностями
    dist_matrix = build_landmark_distances(graph, landmarks, n)
    
    # Решаем задачу с DP
    min_days, routes = solve_with_dp(landmarks, dist_matrix, k, max_distance=20)
    
    # Строим полные маршруты
    daily_routes = build_daily_routes(graph, landmarks, dist_matrix, n, routes)
    
    # Выводим результаты
    print(min_days)
    for route in daily_routes:
        print(len(route))
        print(' '.join(map(str, route)))

if __name__ == "__main__":
    solve()
