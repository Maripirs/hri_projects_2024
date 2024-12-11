#!/usr/bin/python3

import rospy
import tf
from people_msgs.msg import PositionMeasurementArray
import math
from circle_fit import taubinSVD
from weekb.msg import GroupInfo

#Not my code
def find_point(x1, y1, x2, y2, d):
    print(f'person 1 location: {x1}, {y1}')
    print(f'person 2 location: {x2}, {y2}')
    dx = x2 - x1
    dy = y2 - y1
    print(f"Direction vector: dx = {dx}, dy = {dy}")
    magnitude = math.sqrt(dx**2 + dy**2)
    print(f"Magnitude of the direction vector: {magnitude}")
    if magnitude == 0:
        print("Error: The two points are coincident (same location). Cannot extend line.")
        return x2, y2  # Can't extend, return the same point
    unit_dx = dx / magnitude
    unit_dy = dy / magnitude
    print(f"Normalized direction: unit_dx = {unit_dx}, unit_dy = {unit_dy}")
    print (f'Distance {d}')
    extended_dx = unit_dx * d
    extended_dy = unit_dy * d
    print(f"Extended vector: extended_dx = {extended_dx}, extended_dy = {extended_dy}")
    x3 = x2 + extended_dx
    y3 = y2 + extended_dy
    print(f"Extended point: x3 = {x3}, y3 = {y3}")  
    return (x3, y3)

class groupDetector:
    def __init__(self):
        self.pub = rospy.Publisher("/robot0/detected_groups", PositionMeasurementArray, queue_size = 10)
        self.sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.group_callback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )
        self.group_pub = rospy.Publisher("group_info", GroupInfo, queue_size=10)
        self.msg = GroupInfo()
        self.group = "NONE"
        self.counter = 1
        self.people = []
        self.relevant_location = [0,0,0]
        self.circle_r = 0
        rospy.sleep(rospy.Duration.from_sec(0.5))
    
    def group_type(self):
        print("People")
        idx = 1
        point_coordinates = []
        for p in self.people:
            print(f'Person {idx}: {p.name} x:{p.pos.x} y:{p.pos.y}')
            point_coordinates.append([p.pos.x, p.pos.y])
            idx+=1
        if(len(point_coordinates) <3):
            print("We can't see 3 people. No group")
            self.group = "NONE"
            return
        xc, yc, r, sigma = taubinSVD(point_coordinates)
        print(f'Circle center x:{xc}  y:{yc}, radius: {r}')
        if (r < 4):
            print("We have a circle")
            self.group = "Circle"
            self.msg.group_type = "Circle"
            self.msg.x = xc
            self.msg.y = yc
            self.msg.r = r
            self.relevant_location = [xc,yc, 0]
            self.radius = r
        else:
            print("Too big to be a circle")
            if (r < 10):
                print("Too crooked to be a line. No group")
                self.group = "NONE"
            else:
                print("We have a line")
                max_dist = 0
                min_dist = 100
                max_idx = 0
                for i in range(len(point_coordinates)):
                    print(f'person {i} location: {point_coordinates[i][0]}, {point_coordinates[i][1]}')
                    dx = point_coordinates[i][0] - point_coordinates[(i-1) % len(point_coordinates)][0]
                    dy = point_coordinates[i][1] - point_coordinates[(i-1) % len(point_coordinates)][1]
                    dist = math.sqrt( dx*dx + dy*dy )
                    if (dist > max_dist):
                        max_dist = dist
                        max_idx = i
                    if(dist < min_dist):
                        min_dist = dist
                x1 = point_coordinates[max_idx][0]
                x2 = point_coordinates[max_idx -1][0]
                y1 = point_coordinates[max_idx][1]
                y2 = point_coordinates[max_idx -1][1]
                print(f'The people on the outside are: ')
                print(f'person {max_idx} location: {x1}, {y1}')
                print(f'person {max_idx+1} location: {x2}, {y2}')
                print(f'With a distance of {max_dist}')
                spacing = max_dist/len(point_coordinates)
                print(f' Min distance = {min_dist}')
                if(spacing > 2):
                    print("People in line are too spread out. No group")
                    self.group = "NONE"
                elif(min_dist < spacing*0.7):
                    print("Some people are too close together. No group")
                    self.group = "NONE"
                else:
                    print(f'People have an average spacing of {spacing}m. We can try to join the line!')
                    x3, y3 = find_point(x1, y1, x2, y2, spacing + 1)
                    print(f'We will attempt to join the line on location {x3}, {y3}')
                    self.group = "Line"
                    self.relevant_location = [x3, y3, 0]
                    self.msg.group_type = "Line"
                    self.msg.x = x3
                    self.msg.y = y3
                    self.msg.r = 0
        self.group_pub.publish(self.msg)
                        
                
                
    
    def find_person(self, person):
        idx = 0
        for p in self.people:
            dx = person.pos.x - p.pos.x
            dy = person.pos.y - p.pos.y
            dist = math.sqrt( dx*dx + dy*dy )
            print(f'{person.name} is {dist} away from {p.name}')
        
            if dist < 1:

                self.people[idx] = person
                return 0
            idx+=1
        return -1
    
    def group_callback(self, data):
        print(f'Callbacking {self.counter}')
        self.counter += 1
        self.people = []
        print(data.people)
        if len(data.people) == 0:
            return
        person = data.people[0]
        if self.counter > 30:
            self.people = []
            self.counter = 0

        hp = tf.TransformBroadcaster()
        hp.sendTransform(self.relevant_location,tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'group','robot_0/odom')
        for person in data.people:
            person_idx = self.find_person(person)
            if person_idx == -1 and len(self.people) < 3:
            #if len(self.people) < 3:
                self.people.append(person)
        self.group_type()
        
        
    def get_counter(self):
        return self.counter

if __name__ == '__main__':

    rospy.init_node('person_broadcaster')
    
    rate = rospy.Rate(10)
    g = groupDetector()
    while not rospy.is_shutdown():
        g.get_counter()
        rate.sleep()


