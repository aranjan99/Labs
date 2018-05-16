
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

from cozmo.util import degrees, distance_mm, speed_mmps
import asyncio
import time

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
        
    heuristicList = {}
    heuristicCellMap = PriorityQueue()
    heuristicCellMap.put((heuristic(grid.getStart(), grid.getGoals()[0]), 0, (grid.getStart(),0, None)))
    heuristicList[grid.getStart()] = heuristic(grid.getStart(), grid.getGoals()[0])
    
    while heuristicCellMap.qsize() > 0:
        processCell = heuristicCellMap.get()       
        if processCell[2][0] == grid.getGoals()[0]:
            currCell =  processCell[2]
            finalPath = []
            while currCell:
                finalPath.append(currCell[0])
                currCell = currCell[2]
            grid.setPath(finalPath)
            return finalPath.reverse()

        heuristicList.pop(processCell[2][0], None)
        grid.addVisited(processCell[2][0])
        
        for cell in grid.getNeighbors(processCell[2][0]):
            cellHeuristic = heuristic(cell[0], grid.getGoals()[0]) + cell[1] - processCell[1]
            if cell not in grid.getVisited() and (cell[0] not in heuristicList or heuristicList[cell[0]] > cellHeuristic):
                heuristicCellMap.put((cellHeuristic, processCell[1] -  cell[1], (cell[0], cell[1], processCell[2])))
                heuristicList[cell[0]] = cellHeuristic
            
def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    return math.sqrt((goal[1]-current[1])*(goal[1]-current[1]) + (goal[0]-current[0])*(goal[0]-current[0]))

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    cubes = []
    start = grid.getStart()
    goal = False
    updatePath = False
    path = []
    i = 1
    while not stopevent.is_set():
            try:
                cube = robot.world.wait_for_observed_light_cube(timeout=2)
                if cube not in cubes:
                    updatePath = True
                    cubes.append(cube)
                    objects = []
                    for i in range(-1, 2, 1):
                        for j in range(-1, 2, 1):
                            objects.append((int(cube.pose.position.x/grid.scale + start[0] + 1) + i, int(cube.pose.position.y/grid.scale + start[1] + 1) + j))
                    grid.addObstacles(objects);
                    intersect = [list(filter(lambda x: x in objects, sublist)) for sublist in grid.getGoals()]
                    if(intersect):
                        updateGoal = grid.getGoals()[0]
                        grid.clearGoals()
                        grid.addGoal((updateGoal[0]+1, updateGoal[1]+1))
                    if cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
                        grid.clearGoals()
                        r1,r2 = getGoalCoord(cube.pose.rotation.angle_z.degrees)
                        grid.addGoal((int(cube.pose.position.x/grid.scale + start[0] + 1)+r1,  int(cube.pose.position.y/grid.scale + start[1] + 1)+r2))
                        goal = True
                else:
                    updatePath = False
                    if goal == False:               
                        robot.turn_in_place(degrees(30)).wait_for_completed()
            except asyncio.TimeoutError: 
                if goal == False:          
                    robot.turn_in_place(degrees(30)).wait_for_completed()
            if goal == True:
                if updatePath == True:
                    grid.clearStart()
                    grid.setStart((int(robot.pose.position.x/grid.scale + start[0] + 0.5), int(robot.pose.position.y/grid.scale + start[1] + 0.5)))

                    astar(grid, heuristic)
                    path = grid.getPath()
                    print(path)
                    i = 1
                if(i < len(path)):                    
                    print(path[i])
                    curr_angle = robot.pose.rotation.angle_z
                    angle = rotateAngle((path[i][0] - path[i-1][0], path[i][1] - path[i-1][1]))
                    angle_to_turn = angle - curr_angle.degrees
                    print(angle_to_turn)
                    robot.turn_in_place(degrees(angle_to_turn)).wait_for_completed()

                    d = math.sqrt(math.pow(path[1][0] - grid.getStart()[0], 2) + math.pow(path[1][1] - grid.getStart()[1], 2))
                    print(d*grid.scale)
                    robot.drive_straight(distance_mm(d*grid.scale), speed_mmps(20)).wait_for_completed()
                    i=i+1 
                if(i == len(path)):
                    stopevent.set()
                
def rotateAngle(cord):
    cord_list = [(1,0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (-1, -1)]
    angles = [0, 45, 90, 135, 180, -45, -90, -135]
    for i in range(len(cord_list)):
        if cord == cord_list[i]:
            return angles[i]
    return 0        

def getGoalCoord(angle):
    a, b = 0, 0
    if angle >= -22.5 and angle < 67.5:
        a, b = -2, -2
    elif angle >= 67.5 and angle < 157.5:
        a, b = 2, -2
    elif angle > 157.5 or angle <=  -157.5:
        a, b = -2, 2
    elif angle > -157.5 and angle <= -22.5:
        a, b = 2, 2
    return a,b
######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

