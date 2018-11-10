#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from PriorityQueue import PriorityQueue
from map_helper import *
from nav_msgs.srv import GetMap


class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """

        rospy.init_node("a_star")  # start node
        self.planService = rospy.Service('A_Star', GetPlan, self.handle_a_star)
        # Get plan Service contains fields:
        #   start as geometry_msgs/PoseStamped
        #   goal as geometry_msgs/PoseStamped
        #   plan as nav_msgs/Plan
        print 'ready to plan a path using A star'
        self.map = None
        rospy.spin()




    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        print 'handling a star'
        return

       

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

            if current == goal:
                break


            for next in get_neighbors(current, self.map):
                new_cost = cost_so_far[current] + self.euclidean_heuristic(current, goal) + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    frontier.put(next, new_cost)
                    came_from[next] = current

        return came_from

      
    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
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
        pass
  

    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        pass

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        pass


    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
        pass


if __name__ == '__main__':
    astar = A_Star()
    pass
