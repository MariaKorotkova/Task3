# Матрицы вместимости и стоимости рёбер между станциями
edge_caps = [
    [None, 30, 45, 25, 30, 20, 40],
    [30, None, 55, 25, 35, 40, 25],
    [25, 30, None, 45, 75, 30, 40],
    [15, 10, 25, None, 40, 30, 80],
    [10, 45, 15, 60, None, 60, 75],
    [10, 30, 45, 30, 55, None, 40],
    [15, 25, 45, 30, 40, 50, None]
]

edge_costs = [
    [None, 5, 10, 4, 5, 6, 10],
    [1, None, 7, 10, 15, 5, 5],
    [1, 10, None, 5, 4, 7, 12],
    [2, 6, 4, None, 5, 10, 8],
    [1, 7, 4, 4, None, 9, 2],
    [1, 4, 2, 3, 8, None, 12],
    [1, 10, 5, 6, 8, 16, None]
]

# Пропускная способность узлов (станций)
node_caps = [1000, 55, 35, 40, 50, 45, 1000]
n = 7

# Класс ребра графа
class Edge:
    def __init__(self, to, rev, cap, cost):
        self.to = to              # конечная вершина
        self.rev = rev            # индекс обратного ребра в списке смежности
        self.cap = cap            # вместимость ребра
        self.cost = cost          # стоимость единицы потока
        self.flow = 0             # текущий поток

# Класс графа
class Graph:
    def __init__(self, size):
        self.adj = [[] for _ in range(size)]  # список смежности

    def add_edge(self, u, v, cap, cost):
        forward = Edge(v, len(self.adj[v]), cap, cost)
        backward = Edge(u, len(self.adj[u]), 0, -cost)
        self.adj[u].append(forward)
        self.adj[v].append(backward)

# Сопоставление имени узла с его id
def name_to_id(name, mapping):
    if name not in mapping:
        mapping[name] = len(mapping)
    return mapping[name]

node_map = {}
g = Graph(n * 2)  # каждый узел разделяется на вход и выход

# Добавляем рёбра "вход → выход" для каждой станции
for i in range(n):
    in_node = name_to_id(f"n{i+1}-in", node_map)
    out_node = name_to_id(f"n{i+1}-out", node_map)
    g.add_edge(in_node, out_node, node_caps[i], 0)

# Добавляем рёбра между станциями согласно матрицам
for i in range(n):
    for j in range(n):
        if i != j and edge_caps[i][j] is not None and edge_caps[i][j] > 0:
            from_node = name_to_id(f"n{i+1}-out", node_map)
            to_node = name_to_id(f"n{j+1}-in", node_map)
            g.add_edge(from_node, to_node, edge_caps[i][j], edge_costs[i][j])

# Алгоритм минимальной стоимости максимального потока
def min_cost_max_flow(graph, s, t):
    total_flow = 0
    total_cost = 0
    iteration = 0
    INF = 1e9

    while True:
        # Поиск кратчайшего пути (по стоимости) методом Беллмана-Форда
        dist = [INF] * len(graph.adj)
        in_queue = [False] * len(graph.adj)
        prev_node = [-1] * len(graph.adj)
        prev_edge = [-1] * len(graph.adj)
        dist[s] = 0
        in_queue[s] = True
        queue = [s]

        while queue:
            u = queue.pop(0)
            in_queue[u] = False
            for i, e in enumerate(graph.adj[u]):
                if e.cap - e.flow > 0 and dist[e.to] > dist[u] + e.cost:
                    dist[e.to] = dist[u] + e.cost
                    prev_node[e.to] = u
                    prev_edge[e.to] = i
                    if not in_queue[e.to]:
                        queue.append(e.to)
                        in_queue[e.to] = True

        if dist[t] == INF:
            break  # пути больше нет

        # Вычисляем допустимый прирост потока
        aug_flow = INF
        v = t
        while v != s:
            u = prev_node[v]
            e = graph.adj[u][prev_edge[v]]
            aug_flow = min(aug_flow, e.cap - e.flow)
            v = u

        # Печать первых 5 итераций
        if iteration < 5:
            print(f"\nИтерация {iteration + 1}")
            path = []
            v = t
            path_cost = 0
            while v != s:
                u = prev_node[v]
                e = graph.adj[u][prev_edge[v]]
                path.append((u, v, e.cap - e.flow, e.cost))
                path_cost += e.cost
                v = u

            path.reverse()
            path_str = ' → '.join(
                [ [k for k, val in node_map.items() if val == u][0] for u, _, _, _ in path ] +
                [[k for k, val in node_map.items() if val == path[-1][1]][0]]
            )
            print(f"Путь: {path_str}")
            print(f"Пропускаем поток: {aug_flow}")
            print(f"Стоимость на данной итерации: {aug_flow * path_cost}")
            print(f"Общий поток после итерации: {total_flow + aug_flow}")
            print(f"Общая стоимость после итерации: {total_cost + aug_flow * path_cost}")

        # Обновляем потоки
        v = t
        while v != s:
            u = prev_node[v]
            e = graph.adj[u][prev_edge[v]]
            e.flow += aug_flow
            graph.adj[e.to][e.rev].flow -= aug_flow
            total_cost += aug_flow * e.cost
            v = u

        total_flow += aug_flow
        iteration += 1

    print(f"\nВсего итераций: {iteration}")
    return total_flow, total_cost

# Определяем источник и сток
source = name_to_id("n1-in", node_map)
sink = name_to_id("n7-out", node_map)

# Запускаем алгоритм
total_flow, total_cost = min_cost_max_flow(g, source, sink)

print("\nМаксимальное кол-во поездов:", total_flow)
print("Минимальная стоимость проезда максимальным кол-вом поездов:", total_cost, "\n")

# Вывод итоговых потоков
for u in range(len(g.adj)):
    for e in g.adj[u]:
        if e.flow > 0 and e.cost >= 0:
            from_name = [k for k, val in node_map.items() if val == u][0]
            to_name = [k for k, val in node_map.items() if val == e.to][0]
            print(f"{from_name} -> {to_name}: {e.flow}")
