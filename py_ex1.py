import os
import copy
import heapq

INPUT_PATH = os.path.join("input.txt")
OUTPUT_PATH = os.path.join("output.txt")

ALGORITHMS = {1: "IDS", 2: "BFS", 3: "A*"}


class InputParser:
    """
    Parser for the input file
    """

    def __init__(self, file):
        """
        Receives file and parse it according the expected format
        """
        self.file = file
        self.algorithm = ALGORITHMS[int(self.file.readline())]
        self.board_size = int(self.file.readline())
        self.board_init_state = [int(x) for x in self.file.readline().split("-")]


class Node(object):
    """
    Generic node implementation for search problems
    """

    def __init__(self, parent, state, operator):
        """
        :param parent: parent node
        :param state: node's state (implementation is game-dependent)
        :param operator: the operator used to get from parent to this node (U/D/L/R)
        """
        self.parent = parent
        self.state = state
        self.operator = operator

    @property
    def depth(self):
        """
        Calc depth of node by climbing up the tree until no parent node available
        """
        depth = 0
        node = self
        while node.parent:
            depth += 1
            node = node.parent
        return depth

    @property
    def path(self):
        """
        Calc the operator sequence from node to root
        """
        path = ""
        node = self
        while node.parent:
            path = node.operator + path
            node = node.parent
        return path

    @property
    def cost(self):
        """
        Calc total cost of path already calculated + predicted path's heuristic
        """
        return self.state.heuristic + self.depth


class TilesState(object):
    """
    Representation of single state in the Tiles Game
    """

    def __init__(self, board_list, board_size):
        """
        :param board_list: list in the size of board_size X board_size, representing the current game state, as received
               from input
        :param board_size: the size of the board side
        """
        self.board = board_list
        self.board_size = board_size

    @property
    def heuristic(self):
        """
        This function calcs the Manhattan Distance for current game state
        """
        d_sum = 0

        # For every number in the tiles board (1 - board_size^2)
        for i in range(1, self.board_size * self.board_size):
            # Get board_list's current index for i value
            i_list_index = self.board.index(i)

            # Calc tile i current location on board (0 to (board_size - 1))
            cur_y = i_list_index // self.board_size  # Floor function for python3
            cur_x = i_list_index % self.board_size

            # Calc tile i target location on board (0 to (board_size - 1))
            tgt_y = (i - 1) // self.board_size  # Floor function for python3
            tgt_x = (i - 1) % self.board_size

            # Add distance to d_sum
            d_sum += abs(tgt_x - cur_x) + abs(tgt_y - cur_y)
        return d_sum

    def success(self):
        """
        :return: returns true if the game state is winning state - meaning the Manhattan Distance is 0
        """
        return self.heuristic == 0

    def move(self, operator):
        """
        Moves tile up to the blank space, if possible
        """
        blank_index = self.board.index(0)

        if operator == "U":
            moved_tile_index = blank_index + self.board_size
            if moved_tile_index >= len(self.board):  # Out of bounds
                raise Exception("Up movement is forbidden.")

        elif operator == "D":
            moved_tile_index = blank_index - self.board_size
            if moved_tile_index < 0:  # Out of bounds
                raise Exception("Down movement is forbidden.")

        elif operator == "L":
            moved_tile_index = blank_index + 1
            if moved_tile_index % self.board_size == 0:  # Out of bounds
                raise Exception("Left movement is forbidden.")

        elif operator == "R":
            moved_tile_index = blank_index - 1
            if blank_index % self.board_size == 0:  # Out of bounds
                raise Exception("Right movement is forbidden.")

        # Swap Tile with blank space
        self.board[blank_index] = self.board[moved_tile_index]
        self.board[moved_tile_index] = 0


class Game:
    """
    Tiles Game clasa
    """

    def __init__(self, board_list, board_size):
        """
        Initialize board_size, availabe operators & root game node
        :param board_list: list of numbers representing the board initial state, left to right, top to bottom
        :param board_size: size of board side - lenght of board list is board_size * board_size
        """
        self.board_size = board_size
        self.operators = ["U", "D", "L", "R"]

        # Create root node with the recieved board initial state
        self.root_node = Node(None, TilesState(board_list, board_size), None)

    def movement(self, state, operator):
        """
        Moves state according to the operator and return the new state
        """
        new_state = TilesState(copy.copy(state.board), self.board_size)
        new_state.move(operator)
        return new_state


class PriorityQueue(object):
    """
    Priority Queue implementation using heapq - for better runtime
    """
    def __init__(self):
        """
        Initialize PriorityQueue parameters
        """
        self.next_index = 0
        self.heap_items = []

    def is_empty(self):
        """
        Returns true only if PriorityQueue is empty
        """
        return self.next_index == 0

    def push(self, item, priority):
        """
        Inserts item with its priority, If tie - by order of insertion
        """
        heapq.heappush(self.heap_items, (priority, self.next_index, item))
        self.next_index += 1

    def pop(self):
        """
        Pop the item according the priority value. In case of tie - the oldest item in the heap
        """
        return heapq.heappop(self.heap_items)[2]


class Search:
    """
    Search class, containes BFS, IDS & A* alorithms
    """
    def __init__(self, problem):
        """
        Initialize the search variables
        """
        self.problem = problem
        self.nodes = None
        self.opened_node_counter = 0

    def bfs(self):
        """
        BFS implementation
        """
        # Initialize nodes list
        self.nodes = [self.problem.root_node]

        # Open the first node in the list
        cur_node = self.nodes.pop(0)
        self.opened_node_counter += 1

        # Run as long as cur_node isn't goal
        while not cur_node.state.success():
            # For each operator (U/D/L/R)
            for operator in self.problem.operators:
                # Try creating new Node for the state received from implementing the operator on parent,
                # If receiving Exception - continue
                try:
                    self.nodes.append(Node(cur_node, self.problem.movement(cur_node.state, operator), operator))
                except Exception:
                    pass

            # Open the first node in the list
            cur_node = self.nodes.pop(0)
            self.opened_node_counter += 1

        return cur_node

    def ids(self):
        """
        IDS implementation
        """
        # Initialize nodes list & depth
        self.nodes = [self.problem.root_node]
        cur_depth = 0

        while True:
            # Open the first node in the list
            cur_node = self.nodes.pop(0)
            self.opened_node_counter += 1

            # DFS search limited by current depth
            while not cur_node.state.success():
                # If depth requirements met
                if cur_node.depth < cur_depth:
                    # For each operator
                    for operator in reversed(self.problem.operators):
                        # Try creating new Node for the state received from implementing the operator on current node,
                        # If receiving Exception - continue
                        try:
                            self.nodes.insert(0, Node(cur_node, self.problem.movement(cur_node.state, operator), operator))
                        except Exception:
                            pass

                # If node list is empty - no node found on current depth
                if len(self.nodes) == 0:
                    cur_node = None
                    break

                # Else - open next node and retry
                else:
                    # Open the first node in the list
                    cur_node = self.nodes.pop(0)
                    self.opened_node_counter += 1

            # If cur_node is not None - found goal!
            if cur_node:
                return cur_node

            # Else Re-run specific depth search with depth + 1
            cur_depth += 1
            self.opened_node_counter = 0
            self.nodes = [self.problem.root_node]

    def a_star(self):
        # Initialize nodes to Priority Queue
        self.nodes = PriorityQueue()

        # Push start node to queue, cost = g(n) + h(n)
        self.nodes.push(self.problem.root_node, self.problem.root_node.cost)

        # Open next node on PriorityQueue
        cur_node = self.nodes.pop()
        self.opened_node_counter += 1

        # While cur_node isn't goal
        while not cur_node.state.success():
            # For each operator
            for operator in self.problem.operators:
                # Try creating new Node for the state received from implementing the operator on current node,
                # If receiving Exception - continue loop without pushin to PriorityQueue
                try:
                    next_node = Node(cur_node, self.problem.movement(cur_node.state, operator), operator)
                except Exception:
                    continue

                # If next_node is valid - push to PriorityQueue when with cost as priority
                self.nodes.push(next_node, next_node.cost)

            # Open next node on PriorityQueue
            cur_node = self.nodes.pop()
            self.opened_node_counter += 1
        return cur_node


def write_to_output_file(*args):
    """
    :param args: argument to be written in output.txt
    """
    with open(OUTPUT_PATH, "w") as output_file:
        output_file.write(" ".join(map(lambda x: str(x), args)))


def main():
    with open(INPUT_PATH, "r") as input_file:
        parser = InputParser(input_file)

    game = Game(parser.board_init_state, parser.board_size)

    if parser.algorithm == "BFS":
        search = Search(game)
        res_node = search.bfs()
        write_to_output_file(res_node.path, search.opened_node_counter, 0)
    elif parser.algorithm == "A*":
        search = Search(game)
        res_node = search.a_star()
        write_to_output_file(res_node.path, search.opened_node_counter, res_node.cost)
    elif parser.algorithm == "IDS":
        search = Search(game)
        res_node = search.ids()
        write_to_output_file(res_node.path, search.opened_node_counter, res_node.depth)


if __name__ == "__main__":
    main()
