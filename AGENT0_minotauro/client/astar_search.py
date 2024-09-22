import client
import ast
import random
import nodes_h

class Agent:
    def __init__(self):
        self.c = client.Client('127.0.0.1', 50001)
        self.res = self.c.connect()
        random.seed()  # To become true random, a different seed is used! (clock time)
        self.visited_nodes = []
        self.frontier_nodes = []
        self.weight_map =[]
        self.weight_dict = {}
        self.goal_pos =(0,0)
        self.state = (0,0)
        self.max_coord = (0,0)
        self.obstacles = []


    def get_goal_position(self):
        msg = self.c.execute("info", "goal")
        goal = ast.literal_eval(msg)
        # test
        # print('Goal is located at:', goal)
        return goal

    def get_heuristic_value(self,goal: tuple, state: tuple):
        '''This heuristic is calculated by finding the manhattan distance.
        It calculates the difference between coordinates'''
        diff_x = goal[0] - state[0];
        if diff_x <0:
             diff_x = (-1) * diff_x
        diff_y = goal[1] - state[1]
        if diff_y < 0:
             diff_y = (-1) * diff_y

    # Alternative
    #     diff_x1 = goal[0] - state[0]
    #     if diff_x1 < 0:
    #         diff_x1 = (-1) * diff_x1
    #     diff_x2 = self.max_coord[0] - goal[0] + state[0]
    #     if diff_x2 < 0:
    #         diff_x2 = (-1) * diff_x2
    #     if diff_x1 > diff_x2:
    #         diff_x = diff_x1
    #     else:
    #         diff_x = diff_x2
    #     diff_y1 = goal[1] - state[1]
    #     if diff_y1 < 0:
    #         diff_y1 = (-1) * diff_y1
    #     diff_y2 = self.max_coord[1] - goal[1] + state[1]
    #     if diff_y2 < 0:
    #         diff_y2 = (-1) * diff_y2
    #     if diff_y1 > diff_y2:
    #         diff_y = diff_y1
    #     else:
    #         diff_y = diff_y2
        return diff_x + diff_y


    def get_self_position(self):
        msg = self.c.execute("info", "position")
        pos = ast.literal_eval(msg)
        # test
        # print('Received agent\'s position:', pos)
        return pos

    def get_weight_map(self):
        msg = self.c.execute("info", "map")
        w_map = ast.literal_eval(msg)
        # test
        # print('Received map of weights:', w_map)
        return w_map

    def get_weight_dict(self, w_map:list):
        """Transform the weights into a dictionary"""
        w_dict = {}
        for elem in w_map:
            w_dict[elem[0]]=elem[1]
        # print(w_dict)
        return w_dict

    def get_max_coord(self):
        msg = self.c.execute("info","maxcoord")
        max_coord =ast.literal_eval(msg)
        # test
        # print('Received maxcoord', max_coord)
        return max_coord

    def get_obstacles(self):
        msg = self.c.execute("info","obstacles")
        obst =ast.literal_eval(msg)
        # test
        # print('Received map of obstacles:', obst)
        return obst

    def step(self,pos,action):
        """Add new position after an action, using north, east, west or south"""
        if action == "east":
            if pos[0] + 1 < self.max_coord[0]:
                new_pos = (pos[0] + 1, pos[1])
            else:
                new_pos =(0,pos[1])

        if action == "west":
            if pos[0] - 1 >= 0:
                new_pos = (pos[0] - 1, pos[1])
            else:
                new_pos = (self.max_coord[0] - 1, pos[1])

        if action == "south":
            if pos[1] + 1 < self.max_coord[1]:
                new_pos = (pos[0], pos[1] + 1 )
            else:
                new_pos = (pos[0], 0)

        if action == "north":
            if pos[1] - 1 >= 0:
                new_pos = (pos[0], pos[1] - 1)
            else:
                new_pos = (pos[0], self.max_coord[1] - 1 )

        return new_pos

    def mark(self, pos:tuple, colour:str):
        mark_ground = str(pos[0])+","+str(pos[1])+"_"+colour
        msg = self.c.execute("mark",mark_ground)
        obst =ast.literal_eval(msg)

    def unmark(self,pos:tuple):
        unmark_ground = str(pos[0])+","+str(pos[1])
        msg = self.c.execute("unmark",unmark_ground)
        obst =ast.literal_eval(msg)


    def create_node(self,parent_node,action):
        """Create a new node based on action and on parent_node"""
        # return a new position after the actions
        state = self.step(parent_node.get_state(),action)
        return nodes_h.Node(state, parent_node, action, self.weight_dict[state],self.get_heuristic_value(self.goal_pos,state))

    def print_nodes(self,description,nds: list):
        print(description)
        print("Name(Parent):Total cost")
        for nd in nds:
            if nd.get_parent() != None:
                print(nd.get_state(),"(",nd.get_parent().get_state(),"):",nd.get_f())
            else:
                print(nd.get_state(),"(---):",nd.get_f())
        print("---------------------------")

    def not_in_visited_node(self, n_nd:nodes_h.Node, visited:list):
        not_in = True
        for nd in visited:
            if nd.get_state() == n_nd.get_state():
                not_in = False
                break
        return not_in

    def not_an_obstacle(self, n_nd:nodes_h.Node):
        not_obst = True

        for obst in self.obstacles:
            if n_nd.get_state() == obst:
                not_obst = False
                break
        return not_obst

    def remove_nodes_visited(self,front_n:list, visit_n:list):
        new_front_n = []
        for nd in front_n:
            if self.not_in_visited_node(nd,visit_n):
                new_front_n.append(nd)
        return new_front_n

    def expand(self, type:str):
        if type == "bread-first":
            ne = self.frontier_nodes.pop(0)
        if type == "astar":
            self.frontier_nodes.sort(key = lambda nd: nd.get_f())
            ne = self.frontier_nodes.pop(0)
            return ne

    def print_path(self, node):
        n = node
        n_list = []
        while n.get_path_cost() != 0:
            n_list.insert(0,[n.get_state(), n.get_f()])
            n = n.get_parent()
        print("Final Path", n_list)

    def run(self):
        # Get the position of the Goal
        self.goal_pos = self.get_goal_position()
        print("Goal node:",self.goal_pos)

        # Get information of the weights for each step in the world ...
        self.weight_map = self.get_weight_map()
        self.weight_dict = self.get_weight_dict(self.weight_map)
        # Test
        print("weights dict:",self.weight_dict)
        self.obstacles = self.get_obstacles()
        # Test
        print("obstacles:", self.obstacles)
        # Get max coordinates
        self.max_coord = self.get_max_coord()
        # Get the initial position of the agent
        self.state = self.get_self_position()
        # Test heuristic evaluation function
        h = self.get_heuristic_value(self.goal_pos,self.state)
        # Test
        print("Heuristic between goal:",self.goal_pos," and state:",self.state," is:",h)

        # Start thinking
        end = False
        found = None
        #Add first node (root)
        root = nodes_h.Node(self.state,None,"",0,self.get_heuristic_value(self.goal_pos,self.state))
        self.visited_nodes.append(root)
        # Get the first four nodes. They are not in the same position of the root node.
        for act in ["north","east","south","west"]:
            nd = self.create_node(root,act)
            if self.not_an_obstacle(nd):
                self.frontier_nodes.append(nd)
            print("State:",nd.get_state())
            self.mark(nd.get_state(),"red")
        # Test
        # self.print_nodes("Nodes in frontier", self.frontier_nodes)
        # self.print_nodes("Nodes visitied", self.visited_nodes)
        # Cycle expanding nodes following the sequence in frontier nodes.
        while end == False:
            # Expanding strategy
            node_to_expand = self.expand("astar")
            #node_to_expand = self.frontier_nodes.pop(0)
            self.unmark(node_to_expand.get_state())
            self.state = node_to_expand.get_state()
            # Test
            print("Expanding node position:", self.state)
            self.visited_nodes.append(node_to_expand)
            self.mark(node_to_expand.get_state(),"blue")
            for act in ["north","east","west","south"]:
                new_node = self.create_node(node_to_expand,act)
                if self.not_in_visited_node(new_node,self.visited_nodes) and self.not_an_obstacle(new_node):
                    self.frontier_nodes.append(new_node)
                    self.mark(new_node.get_state(),"red")
            # remove all nodes visited from frontier nodes
            self.frontier_nodes = self.remove_nodes_visited(self.frontier_nodes, self.visited_nodes)
            # test
            self.print_nodes("Frontier", self.frontier_nodes)
            self.print_nodes("Visitied", self.visited_nodes)
            for node in self.visited_nodes:
                if node.get_state() == self.goal_pos:
                    print("Node state:",node.get_state())
                    print("GoalNodePos",self.goal_pos)
                    found = node
                    self.print_path(found)
                    end = True
                    break
        input("Waiting for return!")


#STARTING THE PROGRAM:
def main():
    agent = Agent()
    agent.run()

main()