#!/usr/bin/env python
import copy
import numpy
import rospy

from util import *
from rules import *
from db_manager import DBManager

from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse

from social_msgs.msg import People, Person
from social_msgs.msg import Objects, Objects
from social_msgs.msg import Locals, Local
from social_msgs.msg import PointArray
from social_msgs.msg import Destination
from social_msgs.srv import DestinationArray, DestinationArrayResponse

class SocialReasoning(object):
    """docstring for SocialReasoning."""
    def __init__(self):

        self.rate = rospy.Rate(25)
        self.interaction_distance = 2.5
        self.interaction_point_distance = 0.1

        self.locals_data = []
        self.locals = []
        self.people_data = []
        self.people = []
        self.objects_data = []
        self.objects = []

        # self.leg_data = []

        # get database
        db_path_input = rospy.get_param('~database_input')
        db_path_output = '{}.out.db'.format(db_path_input)
        self.db_manager = DBManager(db_path_input, db_path_output)

        # Subscribers
        init_subscriber('locals_data', Locals, self.callback_locals)
        init_subscriber('people_data', People, self.callback_people)
        init_subscriber('objects_data', Objects, self.callback_objects)
        # init_subscriber('leg_data', People, self.callback_leg)
        rospy.loginfo('All subscribers ready.')

        # Publishers
        self.pub_locals = init_publisher('locals', Locals)
        self.pub_people = init_publisher('people', People)
        self.pub_objects = init_publisher('objects', Objects)
        self.pub_marker = init_publisher('social_marker', Marker)
        self.pub_pts_people = init_publisher('points_people', PointArray)
        self.pub_pts_interaction = init_publisher('points_interaction', PointArray)
        rospy.loginfo('All publishers ready.')

        # servive servers
        rospy.Service('~get_destination', DestinationArray, self.handle_getdestination_srv)
        rospy.Service('~clear_people', Empty, self.handle_clearpeople_srv)
        rospy.loginfo('All service servers ready.')

        self.start()

    def callback_locals(self, data):
        self.locals_data = data.locals

    def callback_people(self, data):
        self.people_data = data.people

    def callback_objects(self, data):
        self.objects_data = data.objects

    # def callback_leg(self, data):
    #     self.leg_data = data.people

    def handle_getdestination_srv(self,req):
        dar = DestinationArrayResponse()

        # locals = self.db_manager.get_locals_by_name(req.name)
        locals = [local for local in self.locals if local.name == req.name]
        for local in locals:
            d = Destination()
            d.name = local.name
            d.type = 'local'
            d.pose = local.pose
            dar.destination_list.append(d)

        # people = self.db_manager.get_people_by_name(req.name)
        people = [person for person in self.people if person.name == req.name]
        for person in people:
            d = Destination()
            d.name = person.name
            d.type = 'person'
            d.pose = person.pose
            dar.destination_list.append(d)

        return dar

    def handle_clearpeople_srv(self,req):
        er = EmptyResponse()
        self.db_manager.clear_people()
        return er

    def update_database(self):

        # self.locals = self.db_manager.update_locals(self.locals_data)
        # self.objects = self.db_manager.update_objects(self.objects_data)
        # self.people = self.db_manager.update_people(self.people_data)
        #
        # self.locals = self.db_manager.update_locals(self.locals)
        # self.objects = self.db_manager.update_objects(self.objects)
        # self.people = self.db_manager.update_people(self.people)
        #
        # self.locals_data = []
        # self.objects_data = []
        # self.people_data = []

        self.locals = self.locals_data
        self.objects = self.objects_data
        self.people = self.people_data
        for person in self.people:
            person.pose_approach = get_approach_pose(person)
            person.proxemic = get_proxemic()


    def publish(self):

        # locals
        msg_l = Locals()
        msg_l.header = Header(0,rospy.Time.now(),"/map")
        msg_l.locals = self.locals
        self.pub_locals.publish(msg_l)

        # people
        msg_p = People()
        msg_p.header = Header(0,rospy.Time.now(),"/map")
        msg_p.people = self.people
        self.pub_people.publish(msg_p)

        # objects
        msg_o = Objects()
        msg_o.header = Header(0,rospy.Time.now(),"/map")
        msg_o.objects = self.objects
        self.pub_objects.publish(msg_o)


        msg_pts_people = PointArray()
        msg_pts_interaction = PointArray()

        # people_markes
        for idx, person in enumerate(self.people):
            # body
            msg_marker_body = create_marker(
                _ns="people_body",
                _id=idx,
                _type=Marker.CYLINDER,
                _action=Marker.ADD,
                _scale=Point(0.2, 0.2, 1.0),
                _color=ColorRGBA(1.0,0.0,0.0,1.0))
            pose_body = copy.deepcopy(person.pose)
            pose_body.position.z -= 0.3
            msg_marker_body.pose = pose_body
            self.pub_marker.publish(msg_marker_body)
            # head
            msg_marker_head = create_marker(
                _ns="people_head",
                _id=idx,
                _type=Marker.ARROW,
                _action=Marker.ADD,
                _scale=Point(0.2, 0.2, 0.2),
                _color=ColorRGBA(1.0,0.0,0.0,1.0))
            pose_head = copy.deepcopy(person.pose)
            pose_head.position.z += 0.4
            msg_marker_head.pose = pose_head
            self.pub_marker.publish(msg_marker_head)
            p = Point()
            p = person.pose.position
            msg_pts_people.points.append(p)

        # objects_markes
        msg_pts_o = PointArray()
        for idx, object in enumerate(self.objects):
            msg_marker_obj = create_marker(
                _ns="objects",
                _id=idx,
                _type=Marker.SPHERE,
                _action=Marker.ADD,
                _scale=Point(0.25, 0.25, 0.25),
                _color=ColorRGBA(1.0,1.0,0.0,1.0))
            pose = copy.deepcopy(object.pose)
            msg_marker_obj.pose = pose
            self.pub_marker.publish(msg_marker_obj)

        # locals_markes
        msg_pts_l = PointArray()
        for idx, local in enumerate(self.locals):

            msg_marker_loc_cylinder = create_marker(
                _ns="locals_cylinder",
                _id=idx,
                _type=Marker.CYLINDER,
                _action=Marker.ADD,
                _scale=Point(1.0, 1.0, 0.1),
                _color=ColorRGBA(0.0,1.0,0.0,0.3))
            msg_marker_loc_arrow = create_marker(
                _ns="locals_arrow",
                _id=idx,
                _type=Marker.ARROW,
                _action=Marker.ADD,
                _scale=Point(0.5, 0.1, 0.1),
                _color=ColorRGBA(0.0,1.0,0.0,0.5))
            msg_marker_loc_cylinder.pose = copy.deepcopy(local.pose)
            msg_marker_loc_arrow.pose = copy.deepcopy(local.pose)
            self.pub_marker.publish(msg_marker_loc_cylinder)
            self.pub_marker.publish(msg_marker_loc_arrow)


        id = 0
        for id_o, object in enumerate(self.objects):
            for id_p, person in enumerate(self.people):

                p1 = person.pose.position
                p2 = object.pose.position
                q1 = person.pose.orientation
                [yaw,_,_] = quaternion_to_euler(q1.x, q1.y, q1.z, q1.w)
                dist = numpy.sqrt(pow(p2.x-p1.x,2) + pow(p2.y-p1.y,2))
                ang = numpy.arctan2(p2.y-p1.y, p2.x-p1.x)

                if(dist > self.interaction_distance):
                    continue
                id += 1

                marker_interaction = create_marker(
                    _ns="interaction_object",
                    _id=id,
                    _type=Marker.SPHERE_LIST,
                    _action=Marker.ADD,
                    _scale=Point(0.1, 0.1, 0.0),
                    _color=ColorRGBA(1.0,1.0,0.0,1.0))
                if(abs(ang-yaw) < 10*3.1415/180):
                    cont = 0
                    while(cont < dist):
                        dx = cont * numpy.cos(ang)
                        dy = cont * numpy.sin(ang)
                        cont += self.interaction_point_distance
                        p_interacao = Point()
                        p_interacao.x = p1.x + dx
                        p_interacao.y = p1.y + dy
                        p_interacao.z = (p1.z + p2.z)/2
                        marker_interaction.points.append(p_interacao)
                        msg_pts_interaction.points.append(p_interacao)
                self.pub_marker.publish(marker_interaction)


        id = 0
        for people1 in self.people:
            for people2 in self.people:

                p1 = people1.pose.position
                p2 = people2.pose.position
                q1 = people1.pose.orientation
                [yaw,_,_] = quaternion_to_euler(q1.x, q1.y, q1.z, q1.w)
                dist = numpy.sqrt(pow(p2.x-p1.x,2) + pow(p2.y-p1.y,2))
                ang = numpy.arctan2(p2.y-p1.y, p2.x-p1.x)

                if(dist > self.interaction_distance):
                    continue
                id += 1

                marker_interaction = create_marker(
                    _ns="interaction_people",
                    _id=id,
                    _type=Marker.SPHERE_LIST,
                    _action=Marker.ADD,
                    _scale=Point(0.1, 0.1, 0.0),
                    _color=ColorRGBA(1.0,1.0,0.0,1.0))
                if(abs(ang-yaw) < 10*3.1415/180):
                    cont = 0
                    while (cont < dist):
                        dx = cont * numpy.cos(ang)
                        dy = cont * numpy.sin(ang)
                        cont += self.interaction_point_distance
                        p_interacao = Point()
                        p_interacao.x = p1.x + dx
                        p_interacao.y = p1.y + dy
                        p_interacao.z = (p1.z + p2.z)/2
                        marker_interaction.points.append(p_interacao)
                        msg_pts_interaction.points.append(p_interacao)
                self.pub_marker.publish(marker_interaction)

        self.pub_pts_people.publish(msg_pts_people)
        self.pub_pts_interaction.publish(msg_pts_interaction)

    def start(self):
        while not rospy.is_shutdown():
            self.update_database()
            self.publish()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('SocialReasoning')
        SocialReasoning()
    except KeyboardInterrupt:
        pass
