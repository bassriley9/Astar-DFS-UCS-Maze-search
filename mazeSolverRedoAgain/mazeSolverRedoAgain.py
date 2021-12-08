import heapq
import sys
import operator
import time
import itertools
#from search import  uniform_cost_search
import numpy as np
import utils



# This class represent a node
class Node:
    # Initialize the class
    def __init__(self, position:(), parent:()):
        self.position = position
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # sum of distance
        self.traveled = 0 # nodes checked
        self.weights = 0
        self.edges = 0
        self.cells = []


    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))
    def get_cost(self, fromNode, toNode):
        return self.weights [(fromNode + toNode)]
    def neighbors(self, node):
        return self.edges[node]

# Draw a grid
def draw_grid(maze, width, height, spacing=2, **kwargs):
    for y in range(height):
        for x in range(width):
            print('%%-%ds' % spacing % draw_tile(maze, (x, y), kwargs), end='')
        print()

# Draw a tile
def draw_tile(maze, position, kwargs):
    
    # Get the maze value
    value = maze.get(position)
    #path
    if 'path' in kwargs and position in kwargs['path']: value = '*'
    #start point
    if 'start' in kwargs and position == kwargs['start']: value = 'P'
    #end point
    if 'goal' in kwargs and position == kwargs['goal']: value = '.'
    return value 

# Depth-first search (DFS)
def depth_first_search(maze, start, end):
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    while len(open) > 0:
        
        # Get the last node 
        current = open.pop(-1)
        # Add the current node to the closed list
        closed.append(current)
        
        # Check if we have reached the goal, return the path
        if current == goal_node:
            path = []
            while current != start_node:
                path.append(current.position)
                current = current.parent
            # Return reversed path
            return path[::-1]
        # Unzip the current node position
        (x, y) = current.position
        # Get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        # Loop neighbors
        for next in neighbors:
            # Get value from maze
            maze_value = maze.get(next)
            # Check if the node is a wall
            if(maze_value == '%'):
                continue
            # Create a neighbor node
            neighbor = Node(next, current)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            if (neighbor not in open):
                open.append(neighbor)
         
                
        #max tree depth searched
        print("nodes checked \n")
        print(len(closed))
        #check fringe nodes First in first out
        print("Fringe nodes(nodes left open)  \n")
        print(len(open))

    return None

# A Star Search (A*)
def A_Star(maze, start, end):
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list
        open.sort()
        current = open.pop(0)
        closed.append(current)
        
        #see if current pos = goal pos
        if current == goal_node:
            path = []
            while current != start_node:
                path.append(current.position)
                current = current.parent
            #trace past pathway
            return path[::-1]

        (x, y) = current.position
        # Get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        # Loop neighbors
        for next in neighbors:
            # Get value from maze
            maze_value = maze.get(next)
            # Check if the node is a wall
            if(maze_value == '%'):
                continue
            # Create a neighbor node
            neighbor = Node(next, current)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # costs
            neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
            neighbor.f = neighbor.g + neighbor.h
            
            # pick neighbor based on the lowest f cost
            if(pickBestNeighbor(open, neighbor) == True):
                open.append(neighbor)
                total_F = neighbor.f + current.f
        
        #total F Cost so far
        print("F Cost so Far: ")
        print(total_F)
        #max tree depth searched
        print("Max Nodes Checked: ")
        print(len(closed))
        #check fringe nodes First in first out
        print("Fringe Nodes(nodes left open): ")
        print(len(open))
    return None

# return lowest cost neighbor
def pickBestNeighbor(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True

#repurposing A_Star to run using multiple dicionaries
def MultiCost(maze, start, end):
    NEWF = 0
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    
    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list
        open.sort()
        current = open.pop(0)
        closed.append(current)
        
        #see if current pos = goal pos
        if current == goal_node:
            path = []
            while current != start_node:
                path.append(current.position)
                current = current.parent
            #trace past pathway
            return path[::-1]

        (x, y) = current.position
        # Get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        # Loop neighbors
        for next in neighbors:
            # Get value from maze
            maze_value = maze.get(next)
            # Check if the node is a wall
            if(maze_value == '%'):
                continue
            # Create a neighbor node
            neighbor = Node(next, current)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # costs
            neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            neighbor.h = int(abs(neighbor.position[0] - goal_node.position[0])) + int(abs(neighbor.position[1] - goal_node.position[1]))
            neighbor.f = neighbor.g + neighbor.h
            
            # pick neighbor based on the lowest f cost
            if(pickBestNeighbor(open, neighbor) == True):
                open.append(neighbor)
                total_F = neighbor.f + current.f + NEWF
                totalF = total_F
        #total F Cost so far
        print("F Cost so Far: ")
        print(totalF)
        #max tree depth searched
        print("Max Nodes Checked: ")
        print(len(closed))
        #check fringe nodes First in first out
        print("Fringe Nodes(nodes left open): ")
        print(len(open))
    return None
    
def sum(x, y):
    return (x[0] + y[0])
     
# use c1 = 1/2^x and c2 = 2^x to check the left and right sides

# The main entry point for this module
def main():
    next = False
    # Get a maze (grid)
    maze = {}
    mazeL = []
    checkPoints = []
    chars = ['c']
    start = None
    end = None



    ends = [None]
    width = 0
    height = 0
    # Open a file
    fu = open('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/bigMaze.lay')
    fl = open('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/mediumMaze.lay')
    fk = open('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/smallMaze.lay')
    fo = open ('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/openMaze.lay')
    fi = open ('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/smallSearch.lay')
    fj = open ('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/tinySearch.lay')
    fn = open ('C:/Users/bassr/Desktop/Junior year/Intro to AI/PA1_Search/trickySearch.lay')
    #maze dict for a_star and DFS
    #maze list for uniform cost
    print("Which Maze would you like to check? \n ")
    a2 = input(" A for Large \n B for medium \n C for small \n D for open \n E for Small Search \n F for Tiny Search \n G for Tricky Search \n")
    if(a2 == "A"):
        file = fu;
        #Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
               # mazeL[(x, height)] = chars[x]
                maze[(x, height)] = chars[x]
                if(chars[x] == 'P'):
                    start = (x, height)
                elif(chars[x] == '.'):

                    end = (x, height)
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1

        # Close the file pointer
        file.close()
    elif (a2 == "B"):
        file = fl
                #Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
               # mazeL[(x, height)] = chars[x]
                maze[(x, height)] = chars[x]
                if(chars[x] == 'P'):
                    start = (x, height)
                elif(chars[x] == '.'):

                    end = (x, height)
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1

    elif (a2 == "C"):
        file = fk
                #Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
               # mazeL[(x, height)] = chars[x]
                maze[(x, height)] = chars[x]
                if(chars[x] == 'P'):
                    start = (x, height)
                elif(chars[x] == '.'):

                    end = (x, height)
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1

    elif (a2 == "D"):
        file = fo
                #Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
                #mazeL[(x, height)] = chars[x]
                maze[(x, height)] = chars[x]
                if(chars[x] == 'P'):
                    start = (x, height)
                elif(chars[x] == '.'):

                    end = (x, height)
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1

    elif(a2 == "E"):
        thingE = True
        file = fi
        num = 0
         # Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
                maze[(x, height)] = chars[x]
                #mazeL[(x, height)] = chars[x]
                
                if(chars[x] == 'P'):
                    start = (x, height)
                    
                elif(chars[x] == '.'):
                    end = (x, height)
                    ends.insert(x, end)
                    num = num+1
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1
                #ensure that we are inserting the position into the goal as a node
        #17 checkpoints
        goal1 = ends[1]
        #ends.pop()
        goal2 = ends[2]
        #ends.pop()
        goal3 = ends[3]
        #ends.pop()
        goal4 = ends[4]
        #ends.pop()
        goal5 = ends[5]
        #ends.pop()
        goal6 = ends[6]
        #ends.pop()
        goal7 = ends[7]
        #ends.pop()
        goal8 = ends[8]
        #ends.pop()
        goal9 = ends[9]
        #ends.pop()
        goal10 = ends[10]
        #ends.pop()
        goal11 = ends[11]
        #ends.pop()
        goal12 =ends[12]
        #ends.pop()
        goal13 =ends[13]
        #ends.pop()
        goal14 = ends[14]
        #ends.pop()
        goal15 = ends[15]
        #ends.pop()
        goal16 = ends[16]
        goal17 = ends[17]

        path1 = MultiCost(maze, start, goal1)
        path2 = MultiCost(maze, goal1, goal2)
        path3 = MultiCost(maze, goal2, goal3)
        path4 = MultiCost(maze, goal3, goal4)
        path5 = MultiCost(maze, goal4, goal5)
        path5 = MultiCost(maze, goal5, goal6)
        path6 = MultiCost(maze, goal6, goal7)
        path7 = MultiCost(maze, goal7, goal8)
        path8 = MultiCost(maze, goal8, goal9)
        path9 = MultiCost(maze, goal9, goal10)
        path10 = MultiCost(maze, goal10, goal11)
        path11 = MultiCost(maze, goal11, goal12)
        path12 = MultiCost(maze, goal12, goal13)
        path13 = MultiCost(maze, goal13, goal14)
        path14 = MultiCost(maze, goal14, goal15)
        path15 = MultiCost(maze, goal15, goal16)
        path16 = MultiCost(maze, goal16, goal17)
        path = path1 + path2 + path3 + path4 + path5 + path6 + path7 + path8 + path9 + path10+ path11+ path12+ path13+ path14+ path15 + path16

        print()
        draw_grid(maze, width, height, spacing =1, path =path, start = start, goal = goal17)
        print() 
        ends.pop()
        print("Nodes Expanded: {0}".format(len(path)))
        print()

    elif(a2 == "F"):
        thingF = True
        file = fj
                #Loop until there is no more lines
        num = 0
         # Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
                maze[(x, height)] = chars[x]
                #mazeL[(x, height)] = chars[x]
                
                if(chars[x] == 'P'):
                    start = (x, height)
                    
                elif(chars[x] == '.'):
                    end = (x, height)
                    ends.insert(x, end)
                    num = num+1
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1


        #10 checkpoints
        goal1 = ends[1]
        #ends.pop(0)
        goal2 = ends[2]
        #ends.pop()
        goal3 = ends[3]
        #ends.pop()
        goal4 = ends[4]
        #ends.pop()
        goal5 = ends[5]
        #ends.pop()
        goal6 = ends[6]
        #ends.pop()
        goal7 = ends[7]
        #ends.pop()
        goal8 = ends[8]
        #ends.pop()
        goal9 = ends[9]
        goal10 = ends[10]

        path1 = MultiCost(maze, start, goal1)
        path2 = MultiCost(maze, goal1, goal2)
        path3 = MultiCost(maze, goal2, goal3)
        path4 = MultiCost(maze, goal3, goal4)
        path5 = MultiCost(maze, goal4, goal5)
        path5 = MultiCost(maze, goal5, goal6)
        path6 = MultiCost(maze, goal6, goal7)
        path7 = MultiCost(maze, goal7, goal8)
        path8 = MultiCost(maze, goal8, goal9)
        path9 = MultiCost(maze, goal9, goal10)

        path = path1 + path2 + path3 + path4 + path5 + path6 + path7 + path8 + path9
        print()
        draw_grid(maze, width, height, spacing =1, path =path, start = start, goal = goal10)
        print() 
        ends.pop()
        print("Nodes Expanded: {0}".format(len(path)))
        print()

    elif (a2 == "G"):
        thingG = True
        thingF = False
        file = fn
        num = 0
         # Loop until there is no more lines
        while len(chars) > 0:
         # Get chars in a line
            chars = [str(i) for i in file.readline().strip()]
         # Calculate the width
            width = len(chars) if width == 0 else width
          # Add chars to maze
            for x in range(len(chars)):
                maze[(x, height)] = chars[x]
                #mazeL[(x, height)] = chars[x]
                
                if(chars[x] == 'P'):
                    start = (x, height)
                    
                elif(chars[x] == '.'):
                    end = (x, height)
                    ends.insert(x, end)
                    num = num+1
            # Increase the height of the maze
            if(len(chars) > 0):
                height += 1

        #13 checkpoints
        goal1 = ends[1]
        #ends.pop(0)
        goal2 = ends[2]
        #ends.pop()
        goal3 = ends[3]
        #ends.pop()
        goal4 = ends[4]
        #ends.pop()
        goal5 = ends[5]
        #ends.pop()
        goal6 = ends[6]
        #ends.pop()
        goal7 = ends[7]
        #ends.pop()
        goal8 = ends[8]
        #ends.pop(
        goal9 = ends[9]
        #ends.pop()
        goal10 = ends[10]
        #ends.pop()
        goal11 = ends[11]
        #ends.pop()
        goal12 = ends[12]
        #ends.pop()
        goal13 =ends[13]
        #ends.pop()

        path1 = MultiCost(maze, start, goal1)
        path2 = MultiCost(maze, goal1, goal2)
        path3 = MultiCost(maze, goal2, goal3)
        path4 = MultiCost(maze, goal3, goal4)
        path5 = MultiCost(maze, goal4, goal5)
        path5 = MultiCost(maze, goal5, goal6)
        path6 = MultiCost(maze, goal6, goal7)
        path7 = MultiCost(maze, goal7, goal8)
        path8 = MultiCost(maze, goal8, goal9)
        path9 = MultiCost(maze, goal9, goal10)
        path10 = MultiCost(maze, goal10, goal11)
        path11 = MultiCost(maze, goal11, goal12)
        path12 = MultiCost(maze, goal12, goal13)
  
        path = path1 + path2 + path3 + path4 + path5 + path6 + path7 + path8 + path9 + path10+ path11+ path12
        print()
        draw_grid(maze, width, height, spacing =1, path =path, start = start, goal = goal13)
        print() 
        ends.pop()
        print("Nodes Expanded: {0}".format(len(path)))
        print()

    print("This function searches a maze and returns several values along with the correct pathway to travel \n")
    a1 = input("Would you like to run a DFS or an A Star Search of the maze? type DFS or A UCS for Uniform Cost or M for MultiGoals \n")
    if( a1 == "DFS"):
        # Find the closest path from start(P) to end(.)
        path = depth_first_search(maze, start, end)
        print()
        draw_grid(maze, width, height, spacing=1, path=path, start=start, goal=end)
        print()
        #display path cost

        #display nodes expanded
        print("Nodes Expanded: {0}".format(len(path)))

        #max size of the fringe

        print()
    elif(a1 == "A"):
        path = A_Star(maze, start, end)
        print()
        draw_grid(maze, width, height, spacing = 1, path = path, start = start, goal = end)
        print()
        print("Nodes Expanded: {0}".format(len(path)))

    #uniform cost search for medium maze
    elif (a1 == "UCS"):
        print("Portion of the code would not run properly \n was removed to create a cleaner final source code\n")

    elif(a1 == "M"):
        return None







# Tell python to run main method
if __name__ == "__main__": main()
