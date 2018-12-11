#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from custom_msgs.msg import *
from custom_msgs.srv import *
import time
import os
from math import *
import sys
from vehicleState import *
from pathPlanning import *
from Point import *
from helper_functions import *

from geometry_msgs.msg import PoseWithCovarianceStamped

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *

import ref_path


PATH_LENGTH_INDEX = 45
PATH_CUTOFF_RATIO = 2.5
OBSTACLE_BACKTRACK_INDEX_DISTANCE = 11
OBSTACLE_LOOKAHEAD_EULER_DISTANCE = 160.0
REFPATH_POINT_DENSITY = 20.0

FEISABLE_MAX_TIME = 4
FEISABLE_MOD_POINT = 6.0
FEISABLE_MOD_THETA = 0.5

FIRST_MOD_POINT = 6.0
FIRST_MOD_THETA = 0.5

MOD_POINT = 3.0
MOD_THETA = 0.3

RPI_FEISABLE_MAX_TIME = 8
RPI_FEISABLE_MOD_POINT = 9.0
RPI_FEISABLE_MOD_THETA = 0.6

RPI_FIRST_MOD_POINT = 9.0
RPI_FIRST_MOD_THETA = 0.6

RPI_MOD_POINT = 9.0
RPI_MOD_THETA = 0.6


MAX_TIME = 15



class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)

        
        if rospy.get_param('path_planning/rpi', False):
            print "Using RPI params"
            FEISABLE_MAX_TIME = RPI_FEISABLE_MAX_TIME
            FEISABLE_MAX_TIME = RPI_FEISABLE_MAX_TIME
            FEISABLE_MOD_POINT = RPI_FEISABLE_MOD_POINT
            FEISABLE_MOD_THETA = RPI_FEISABLE_MOD_THETA

            FIRST_MOD_POINT = RPI_FIRST_MOD_POINT
            FIRST_MOD_THETA = RPI_FIRST_MOD_THETA

            MOD_POINT = RPI_MOD_POINT
            MOD_THETA = RPI_MOD_THETA
        
        #index of approx where on the refpath the current startstate is
        self.current_start_index = 0
        self.done = False

        self.goals = []
        self.goal_indices = []


        self.first_iteration = False

        self.trailer_path = []

        self.map_obj = Map()
        self.ref_obj = ref_path.RefPath()

        self.map, self.scale = self.map_obj.getMapAndScale()
        self.scale = float(self.scale)
        self.pathplanner = PathPlanner(self.map)

        self.refpath = None

        self.wait_for_map_update = False

        self.latest_state = None

        self.current_start_state = None
        
        #path that truck is following at the moment. Also includes full vehicle state for each position
        self.current_path_states = []

        self.active = False


        self.startend_publisher = rospy.Publisher('alg_startend', Path, queue_size=10)
        self.path_append_publisher = rospy.Publisher('path_append', Path, queue_size=10)
        self.path_rework_publisher = rospy.Publisher('path_rework', Path, queue_size=10)
        self.refpath_publisher = rospy.Publisher('ref_path', Path, queue_size=10)
        self.long_path_publisher = rospy.Publisher('long_path', Path, queue_size=10)
        self.trailer_path_publisher = rospy.Publisher('trailer_path', Path, queue_size=10)


        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initPoseCallback)
        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)
        rospy.Subscriber('truck_state', TruckState, self.truckStateHandler)

        self.path_request_srv = rospy.Service('request_path', RequestPath, self.requestPathHandler)


    def initPoseCallback(self, data):
        #called when setting initial pose in rviz, forgot why i put this here 
        self.current_path_states = self.current_path_states[:1]

    def isVehicleStateOK(self, state):
        return self.pathplanner.checkIfInTrack(state)

    def isCurrentPlanOK(self):
        #returns the index of the current path that collides with obstacle
        for i in range(len(self.current_path_states)-1):
            this_state = self.current_path_states[i]
            next_state = self.current_path_states[i+1]
            avg_x = (this_state.x + next_state.x) / 2
            avg_y = (this_state.y + next_state.y) / 2
            avg_t1 = (this_state.theta1 + next_state.theta1)/2
            avg_t2 = (this_state.theta2 + next_state.theta2)/2
            avg_state = VehicleState(avg_x, avg_y, avg_t1, avg_t2)
            
            if (not self.isVehicleStateOK(this_state)) or (not self.isVehicleStateOK(avg_state)) or (not self.isVehicleStateOK(next_state)):
                return (False,i)

        return (True,0)

    def updateMap(self, obst):
        #add/remove obstacle and update map
        added = self.map_obj.addObstacle(obst)
        if not added:
            removed = self.map_obj.removeObstacle(obst)
            if not removed:
                print "Can't add or remove obstacle"

        self.map, _ = self.map_obj.getMapAndScale()
        if added:
            return True
        else:
            return False


    def mapUpdateHandler(self, data):

        
        self.wait_for_map_update = True

        added_obst = self.updateMap(data.data)
        self.pathplanner.setMap(self.map)

        if added_obst:
            print "Added obstacle:", data.data

            (ok, i) = self.isCurrentPlanOK()

            if ok:
                print "Current path OK"


            else:
                print "Current path NOT ok"
                #move back current start some distance before collision
                if i-OBSTACLE_BACKTRACK_INDEX_DISTANCE < 0:
                    self.current_start_state = self.latest_state
                    self.current_path_states = []
                    self.trailer_path = []
                else:
                    self.current_start_state = self.current_path_states[i-OBSTACLE_BACKTRACK_INDEX_DISTANCE]
                    self.current_path_states = self.current_path_states[:i-OBSTACLE_BACKTRACK_INDEX_DISTANCE+1]
                    self.trailer_path = self.trailer_path[:i-OBSTACLE_BACKTRACK_INDEX_DISTANCE+1]

                #rework path in auto_master
                self.publishReworkPath(self.current_path_states)
                
                self.done = False
                
                #match refpath startindex with startstate position
                self.current_start_index = getClosestIndex(self.refpath, self.current_start_state.getPoint())
                

                self.active = True

            

        else:
            
            print "Removed obstacle:", data.data
            if (not self.active) and (not self.done):
                
                #if the last attempt failed (didn't find any path), try again

                self.path_rework_publisher.publish(Path([]))
                s = self.current_start_state = self.latest_state
                self.current_start_index = 0
                self.trailer_path = []

                self.current_path_states = [s.copy()]



                start = ref_path.VehicleState(s.x, s.y, s.theta1, s.theta2)


                ci = getClosestIndex(self.refpath, (s.x, s.y))
                
                #we may have gone passed some goals last attempt, so filter out those with lower index than the current state
                nbr_goals = len(filter(lambda i: i > ci, self.goal_indices))

                rp, self.goal_indices = self.ref_obj.getRefPath(start, self.goals[-nbr_goals:])
                if rp == [] or rp == None:
                    print "Can't find a refernce path"
                    self.active = False
                    return

                self.refpath = rp
                
                self.publishRefPath(self.refpath)
                self.active = True
                self.first_iteration = True

    
    


    def truckStateHandler(self, data):
        self.latest_state = VehicleState(data.p.x / self.scale, data.p.y / self.scale, data.theta1, data.theta2)
        
        self.current_path_states = traversePath(self.latest_state.getPoint(), self.current_path_states)


    def requestPathHandler(self, data):

        self.trailer_path = []
        s = data.state

        start = ref_path.VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)
        self.goals = [(float(p.x)/self.scale, float(p.y)/self.scale) for p in data.goals.path]

        response = RequestPathResponse()

        rp, self.goal_indices = self.ref_obj.getRefPath(start, self.goals)
        if rp == []:
            response.success = False
            response.message = "Couldn't find reference path. Possible cause: start or goal way off"

        else:
            response.success = True
            self.done = False
            self.current_start_index = 0

            self.first_iteration = True

            self.current_start_state = VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)

            self.refpath = rp

            self.publishRefPath(self.refpath)

            self.wait_for_map_update = False

            self.current_path_states = [self.current_start_state.copy()]
            self.active = True


        return response





    def spin(self):
        path_extension = 0
        while not rospy.is_shutdown():

            if not self.active:
                time.sleep(0.05)
            else:
                
                # ---------- Determine subgoal -------------
                path_length = PATH_LENGTH_INDEX + path_extension

                last_section = False
                if self.current_start_index + path_length>= len(self.refpath) - 1:
                    goal_2, goal = self.refpath[-2:]
                    last_section = True
                else:
                    goal_index = self.current_start_index + path_length
                    goal_2, goal = self.refpath[goal_index-1], self.refpath[goal_index]

                    
                    #Check if the subgoal is on or right after an obstacle. If so, move the subgoal further along refpath
                    last_point_on_obstacle = None
                    for j in range(self.current_start_index, self.current_start_index + path_length+1):
                        
                        p1,p2 = self.refpath[j], self.refpath[j+1]
                        pts = getPointsInBetween(p1,p2,6)
                        for x,y in pts:
                            if self.map[y][x] in [0]:
                                last_point_on_obstacle = (x,y)


                    if last_point_on_obstacle != None:
                        dist_to_goal = getDistance(goal, last_point_on_obstacle)
                        if dist_to_goal <= OBSTACLE_LOOKAHEAD_EULER_DISTANCE:
                            added_space_behind = int(round(-1/REFPATH_POINT_DENSITY * dist_to_goal + OBSTACLE_LOOKAHEAD_EULER_DISTANCE / REFPATH_POINT_DENSITY))
                            
                            path_extension += max(added_space_behind, 1)
                            continue


                path_extension = 0

                self.publishStartEnd(self.current_start_state.getPoint(), goal)


                self.wait_for_map_update = False
                self.pathplanner.setOptimalpath(self.refpath[self.current_start_index:self.current_start_index + path_length])

                
                # ------------- GETTING THE PATH ---------------------
                path = self.getPath(self.current_start_state, goal, goal_2, self.first_iteration)
                
                self.first_iteration = False



                if self.wait_for_map_update: #map updated while planning, redo the same call
                    continue


                if path == []:
                    
                    # ------ COULDN'T FIND A PATH, TRY KTH SHORTEST
                
                    # Determine which sections of the current refpath is interesting to try alternative paths for
                    first_sub_goal_index = None
                    last_sub_goal_index = None

                    for i, gi in enumerate(self.goal_indices):
                        if gi > self.current_start_index and gi < self.current_start_index + path_length:
                            if first_sub_goal_index == None:
                                first_sub_goal_index = i
                            last_sub_goal_index = i

                    if first_sub_goal_index == None:
                        for i in range(len(self.goal_indices)-1):
                            gi_1 = self.goal_indices[i]
                            gi_2 = self.goal_indices[i+1]
                            if self.current_start_index >= gi_1 and self.current_start_index <= gi_2:
                                (first_sub_goal_index, last_sub_goal_index) = (i, i+1)
                                break


                    else:
                        
                        if first_sub_goal_index != 0:
                            first_sub_goal_index -= 1

                        if last_sub_goal_index != len(self.goal_indices) -1:
                            last_sub_goal_index += 1

                    alt_path_k = 1
                    while not rospy.is_shutdown():
                        found_alt_rp = False
                        found_feisable_path = False
                        # try each section with same k, starting from the last one. Increase k if no alt paths was feisable
                        for i in range(first_sub_goal_index, last_sub_goal_index)[::-1]: 

                            starti = self.goal_indices[i]
                            stopi = self.goal_indices[i+1]

                            alt_refpath = self.ref_obj.getAltPath(self.refpath, starti, stopi, alt_path_k)
                            if alt_refpath == [] or alt_refpath == None :
                                continue

                            found_alt_rp = True

                            self.publishRefPath(alt_refpath)


                            len_diff = len(alt_refpath) - len(self.refpath)
                            
                            new_optimal_path = alt_refpath[self.current_start_index:stopi + len_diff+1]
                            if len(new_optimal_path) < 2:
                                continue
                            
                            self.pathplanner.setOptimalpath(new_optimal_path)

                            goal_i = stopi + len_diff
                            goal = alt_refpath[goal_i]
                            goal_2 = alt_refpath[goal_i -1]
                            
                            self.publishStartEnd(self.current_start_state.getPoint(), goal)
                            
                            #check if this alt path is feisable or if we still can't find a path
                            self.wait_for_map_update = False
                            while not rospy.is_shutdown():
                                is_feisable = self.checkIfPathFeisable(self.current_start_state, goal, goal_2)
                                if not self.wait_for_map_update:
                                    break
                            
                            if is_feisable:
                                found_feisable_path = True
                                
                                # increase indices to account for difference in length with new refpath 
                                for i in range(i+1, len(self.goal_indices)):
                                    self.goal_indices[i] += len_diff

                                self.refpath = alt_refpath
                                break


                        if found_feisable_path:
                            print "Found feisable alternate path"
                            break
                        if not found_alt_rp:
                            print "Cant find an alternate path"
                            self.active = False
                            break

                        alt_path_k += 1


                    if self.active == False:
                        continue
                    if found_feisable_path:
                        continue




                #cut off path unless final goal reached
                if last_section:
                    self.done = True
                    cutoff_index = len(path)-1
                else:
                    cutoff_index = min(len(path) -1, int(ceil(len(path)/PATH_CUTOFF_RATIO))+1)

                self.current_start_state = path[cutoff_index]
                append_path = path[:cutoff_index+1]
                self.current_path_states += append_path


                self.publishLongPath(path)


                self.publishAppendPath(append_path, self.done)
                self.publishTrailerPath(append_path)
                
                
                if self.done:
                    self.active = False
                    continue

                self.current_start_index += int(round(PATH_LENGTH_INDEX/PATH_CUTOFF_RATIO))


    def getPath(self, startstate, goal_1, goal_2, first_iteration):
        if self.checkIfPathFeisable(startstate, goal_1, goal_2):
            if first_iteration:
                mp, mt = FIRST_MOD_POINT, FIRST_MOD_THETA
            else:
                mp, mt = MOD_POINT, MOD_THETA
            
            return self.pathplanner.getPath(self.current_start_state, goal_1, goal_2, MAX_TIME, mp, mt)
        else:
            return []
    
    def checkIfPathFeisable(self, startstate, goal_1, goal_2):
        return self.pathplanner.getPath(startstate, goal_1, goal_2, FEISABLE_MAX_TIME, FEISABLE_MOD_POINT, FEISABLE_MOD_THETA, returnsIfFeisable=True)
    
    def publishAppendPath(self, statelist, done):
        p = []
        for state in statelist:
            xx = round(state.x * self.scale)
            yy = round(state.y * self.scale)

            p.append(Position(xx,yy))

        if done:
            p.append(Position(-1, -1))

        self.path_append_publisher.publish(Path(p))
        
    def publishLongPath(self, statelist):
        
        lp = []
        for state in statelist:
            lx = round(state.x * self.scale)
            ly = round(state.y * self.scale)
            lp.append(Position(lx, ly))
            
        self.long_path_publisher.publish(lp)

    def publishRefPath(self, path):
        p = [Position(x*self.scale,y*self.scale) for x,y in path]
        self.refpath_publisher.publish(Path(p))
    
    def publishTrailerPath(self, statelist):
        for state in statelist:
                    
            tx = state.x - HEADER_FRONTAXIS_TO_JOINT * cos(state.theta1) - (TRAILER_LENGTH + TL_BACK) * cos(state.theta2)
            ty = state.y - HEADER_FRONTAXIS_TO_JOINT * sin(state.theta1) - (TRAILER_LENGTH + TL_BACK) * sin(state.theta2)
            self.trailer_path.append(Position(round(tx * self.scale),round(ty * self.scale)))

            
        self.trailer_path_publisher.publish(Path(self.trailer_path))

    def publishStartEnd(self, p1, p2):
        sp = Position(p1[0] * self.scale, p1[1] * self.scale)
        ep = Position(p2[0] * self.scale, p2[1] * self.scale)
        self.startend_publisher.publish(Path([sp, ep]))
    
    def publishReworkPath(self, statelist):
        p = []
        for state in statelist:
            xx = round(state.x * self.scale)
            yy = round(state.y * self.scale)
            p.append(Position(xx,yy))
        self.path_rework_publisher.publish(Path(p))



if __name__ == '__main__':
    p = PathPlanningNode()
    p.spin()
