import client
import ast
import random


class Client:
    '''The class interact with the server sending a string encoded and receive a return of 2048 bits.'''
    def __init__(self,HOST='127.0.0.1',PORT=50001):
        self.host = HOST
        self.port = PORT
        self.s = client.Client(HOST,PORT)
        self.s.connect()
    def print_message(self,data):
        print("Data:",data)
    def execute(self,action,value,sleep_t = 0.5):
        return self.s.execute(action,value,sleep_t)


    def getPos(self):
        '''Return the actual position of the agent. '''
        msg = self.s.execute("info", "position")
        pos = ast.literal_eval(msg)
        # test
        #print('Received agent\'s position:', pos)
        return pos

    def getMap(self):
        '''Return the map of weights: A matrix (x,y) with x the columns and y the rows!'''
        msg = self.s.execute("info", "map")
        w_map = ast.literal_eval(msg)
        # test
        #print('Received map of weights:', w_map)
        return w_map

    def getMaxCoord(self):
        msg = self.s.execute("info","maxcoord")
        max_coord =ast.literal_eval(msg)
        # test
        #print('Received maxcoord', max_coord)
        return max_coord

    def getObstacles(self):
        msg = self.s.execute("info","obstacles")
        obst =ast.literal_eval(msg)
        # test
        #print('Received map of obstacles:', obst)
        return obst

    def getNextPositions(self,pos):
        '''Return all the possible next positions of the agent, given the present position of the agent'''
        next_pos = []
        max_coord = self.getMaxCoord()
        if pos[0] + 1 < max_coord[0]:
            next_pos.append((pos[0]+1,pos[1]))
        else:
            next_pos.append((0,pos[1]))
        if pos[1] + 1 < max_coord[1]:
            next_pos.append((pos[0],pos[1]+1))
        else:
            next_pos.append((pos[0],0))
        if pos[0] - 1 >= 0:
            next_pos.append((pos[0]-1,pos[1]))
        else:
            next_pos.append((max_coord[0]-1,pos[1]))
        if pos[1] - 1 >= 0:
            next_pos.append((pos[0],pos[1]-1))
        else:
            next_pos.append((pos[0],max_coord[1]-1))
        return next_pos

    def run(self):
        res = self.getMaxCoord()
        print("Max coord x:",res[0])
        print("Max coord y:",res[1])
        #  Get information of the world
        #  Print the obstacles position
        obstacles = self.getObstacles()
        #Test: To confirm that the first parameter in matrix is column and the sencond is the row.
        print("Obstacles:", obstacles)
        # Get information of the weights for each step in the world ...
        map = self.getMap()
        #Test: The same from above.
        print("Map:", map)
        input("Press a key to stop!")

def main():
    #STARTING PROGRAM
    agent = Client()
    agent.run()

main()