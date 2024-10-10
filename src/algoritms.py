from collections import deque
from utils.colors import Color
from queue import PriorityQueue
import heapq
import itertools
class Algorithms:
    def __init__(self, graph):
        self.found_path = []
        self.visit_order = []
        self.graph = graph
        self.designations = ['Profundidade Primeiro', 'Largura Primeiro', 'Greedy BFS', 'A*', 'Dijkstra']

    def dfs(self, start_node, end_node):
        self.found_path = []
        visited = set()

        def _dfs_recursive(node):
        
            # Veerificar se o nó já foi visitado; se sim, não processar novamente
            #Incluir nó no set visited e self.visited_order(list.append(node))
            #Verificar se o nó é o nó final; se sim, return True
            if node == end_node:
                self.found_path.append(node)
                return True
            if node in visited:
                return False
            visited.add(node)
            self.visit_order.append(node)
            for neighbor in self.graph.get_neighbors(node):
                if _dfs_recursive(neighbor):
                    self.found_path.append(node)
                    return True
            return False
        _dfs_recursive(start_node)
        self.founder_path.reverse()
        

        return	self.visit_order, self.found_path
        

    def bfs(self, start_node, end_node):
        visited = set()
        pred = {}
        queue = []
        queue = deque()
        while queue:
            node = queue.popleft()
            if node == end_node:
                self.found_path.append(node)
                while node != start_node:
                    self.found_path.append(pred[node])
                    node = pred[node]
                self.found_path.reverse()
                return self.visit_order, self.found_path
            if node in visited:
                continue
            visited.add(node)
            self.visit_order.append(node)
            for neighbor in self.graph.get_neighbors(node):
                if neighbor not in visited:
                    pred[neighbor] = node
                    queue.append(neighbor)
        return [], []

    def greedy_bfs(self, start_node, end_node):
        # Fila de prioridade para os nós a serem explorados (entrada)
        entry_queue = []
        # Fila para armazenar os nós já explorados (saída)
        visited = []
        
        # Contador para garantir que os nós com a mesma heurística sejam diferenciados
        counter = itertools.count()

        # Adiciona o nó inicial à fila de entrada (usando o valor heurístico)
        heapq.heappush(entry_queue, (start_node.get_heuristic(), next(counter), start_node))

        # Dicionário para rastrear o nó pai de cada nó (reconstrução do caminho)
        parent_map = {start_node: None}

        while entry_queue:
            # Remove o nó com a menor heurística da fila de entrada
            _, _, current_node = heapq.heappop(entry_queue)

            if current_node in visited:
                continue

            visited.append(current_node)
            self.visit_order.append(current_node)


            # Se o nó atual for o nó final, reconstrói o caminho e retorna
            if current_node == end_node:
                self.found_path = [current_node]
                while current_node != start_node:
                    current_node = parent_map[current_node]
                    self.found_path.insert(0, current_node)
                return self.visit_order, self.found_path


        # Para cada nó vizinho do nó atual
            for neighbor in self.graph.get_neighbors(current_node):
            # Se o vizinho ainda não foi visitado
                if neighbor not in visited:
                # Adiciona o vizinho à fila de entrada
                    heapq.heappush(entry_queue, (neighbor.get_heuristic(), next(counter), neighbor))
                # Atualiza o mapa de pais para reconstruir o caminho depois
                    parent_map[neighbor] = current_node

        # Se o caminho não for encontrado, retorna as ordens de visita e o caminho vazio
        return self.visit_order, self.found_path
    
    def a_star(self, start_node, end_node):
        return [], []

    def dijkstra(self, start_node, end_node):
        return [], []

    def perform_search(self, search_type, start_node, end_node):
        match search_type:
            case "Profundidade Primeiro":
                self.visit_order, self.found_path = self.dfs(start_node, end_node)
            case "Largura Primeiro":
                self.visit_order, self.found_path = self.bfs(start_node, end_node)
            case "Greedy BFS":
                self.visit_order, self.found_path = self.greedy_bfs(start_node, end_node)
            case "A*":
                self.visit_order, self.found_path = self.a_star(start_node, end_node)
            case "Dijkstra":
                self.visit_order, self.found_path = self.dijkstra(start_node, end_node)
        return self.visit_order,self.found_path