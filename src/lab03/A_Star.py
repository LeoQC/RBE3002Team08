#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from PriorityQueue import PriorityQueue
from map_helper import *
from nav_msgs.srv import GetMap
import math


class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """

        rospy.init_node("a_star")  # start node
        self.planService = rospy.Service('A_Star', GetPlan, self.handle_a_star)

        self.costmap = rospy.Publisher("/move_base/global_costmap/costmap", OccupancyGrid, queue_size=2)
        
        self.dynamic_map_client()
        self.paintMap=OccupancyGrid()
        self.paintMap.info =  self.map.info # TODO change to copy later
        self.paintMap.header = self.map.header

        self.wfPub = rospy.Publisher('AstarDisplay/WaveFront', OccupancyGrid, queue_size=2)

        print 'Ready to plan a path using A star'
        self.map = None

        rospy.spin()

    def get_occupancy_grid(self, map):
        self.occupancy_grid = map


    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orientation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        self.dynamic_map_client()

        start = world_to_map(req.start.pose.position.x, req.start.pose.position.y, self.map)
        end = world_to_map(req.goal.pose.position.x, req.goal.pose.position.y, self.map)

        unfinishedPath = self.a_star(start, end)

        finishedPath = self.reconstruct_path(start, end, unfinishedPath)

        optimizedPath = self.optimize_path(finishedPath)

        path = self.publish_path(optimizedPath)  # self.publish_path(finishedPath)

        return path

       

    def dynamic_map_client(self):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
        rospy.wait_for_service('static_map') 
        try:
            currentMap = rospy.ServiceProxy('static_map', GetMap)
            resp = currentMap()
            self.map = resp.map
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        


        while not frontier.empty():
            current = frontier.get()
            # if ( current[0] == goal[0]) and (current[1] == goal[1]):
            if current == goal:
                break

            # print "get Neighbor return \n" ,get_neighbors(current, self.map)
            for next in get_neighbors(current, self.map):
                new_cost = cost_so_far[current] + self.euclidean_heuristic(current, goal) + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    frontier.put(next, new_cost)
                    came_from[next] = current
            # display the frontier
            self.paint_cells(frontier,came_from)
            rospy.sleep(0.005)
            # print "dis and coord in frontiers \n" , frontier.elements

        return came_from

      
    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        xDiff = point1[0] - point2[0] 
        yDiff = point1[1] - point2[1]
        dis = math.sqrt(xDiff*xDiff + yDiff*yDiff)
        return dis
    pass

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        pass


    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
       """

        path = []
        stack = list()

        while came_from.get(goal) != None:
            stack.append(goal)
            goal = came_from.get(goal)
        stack.append(start)

        while len(stack):
            path.append(stack.pop())

        return path
  

    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        x = 0 
        y = 1
        optPath = [path[0]]
        for ii in range (1, len(path)-1):
            past =optPath[-1]
            current = path[ii]
            future = path[ii+1]

            if (past[x] == current[x] == future[x]) or (past[y] == current[y] == future[y]):
                # move forward once for current and future
                pass
            else:
                # current is the corner, store it, than move on.
                optPath.append(current)
        optPath.append(path[-1])
        return optPath

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        newCells =[0] * len(self.map.data)
        width = self.map.info.width 




        # if came_from.keys() != None:
        for cameKey in came_from.keys():
            x = cameKey[0]
            y= cameKey[1]
            newCells[x + y * width] = 30
            # if not came_from.get(cameKey) == None:
            #     x= came_from.get(cameKey)[0]
            #     y= came_from.get(cameKey)[1]
            # newCells[x+y*width] = 30 # waveCell[0]

        for waveCell in frontier.elements:

            x = waveCell[1][0] ; y = waveCell[1][1]
            newCells[x+y*width] = 50 # waveCell[0]


        self.paintMap.data=newCells
        self.wfPub.publish(self.paintMap)

    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path (in mapspace)
            :return: Path() (in world space)
        """

        path = Path()
        path.header.frame_id = 'map'
        posList = []
        for point in points:
            worldPoint = map_to_world(point[0], point[1], self.map)
            print(worldPoint)
            pos = PoseStamped()
            pos.pose.position.x = worldPoint[0]
            pos.pose.position.y = worldPoint[1]
            posList.append(pos)

        path.poses = posList

        return path


if __name__ == '__main__':
    astar = A_Star()
    pass
