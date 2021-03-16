#!/usr/bin/env python
import rospy
import shutil
import sqlite3

from geometry_msgs.msg import Pose

from social_msgs.msg import Person
from social_msgs.msg import Object
from social_msgs.msg import Local

from rules import *

class DBManager:
    """docstring for db_manager."""
    def __init__(self, _db_path_input, _db_path_output):
        self.db_path_input = _db_path_input
        self.db_path_output = _db_path_output
        shutil.copyfile(self.db_path_input,self.db_path_output)
        rospy.loginfo('Using "{}" database.'.format(self.db_path_output))

    def clear_locals(self):
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        query = '''DELETE FROM locals;'''
        cursor.execute(query)

        db.commit()
        db.close()

    def clear_objects(self):
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        query = '''DELETE FROM objects;'''
        cursor.execute(query)

        db.commit()
        db.close()

    def clear_people(self):
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        query = '''DELETE FROM person;'''
        cursor.execute(query)

        db.commit()
        db.close()
        return locals

    def get_locals_by_name(self, _name):
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        query = '''
            SELECT pose FROM locals
            WHERE name = '{name}';
            '''
        cursor.execute(query.format(name=_name))
        result = cursor.fetchall()
        qtde = len(result)
        poses = [row[0].encode("utf-8") for row in result]

        locals = []
        for pose in poses:
            s = pose.split()
            local = Local()
            local.name = _name
            local.pose.position.x = float(s[0])
            local.pose.position.y = float(s[1])
            local.pose.position.z = float(s[2])
            local.pose.orientation.x = float(s[3])
            local.pose.orientation.y = float(s[4])
            local.pose.orientation.z = float(s[5])
            local.pose.orientation.w = float(s[6])
            locals.append(local)

        db.commit()
        db.close()
        return locals

    def get_people_by_name(self, _name):
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        query = '''
            SELECT pose FROM person
            WHERE name = '{name}';
            '''
        cursor.execute(query.format(name=_name))
        result = cursor.fetchall()
        qtde = len(result)
        poses = [row[0].encode("utf-8") for row in result]

        people = []
        for pose in poses:
            s = pose.split()
            person = Person()
            person.name = _name
            person.pose.position.x = float(s[0])
            person.pose.position.y = float(s[1])
            person.pose.position.z = float(s[2])
            person.pose.orientation.x = float(s[3])
            person.pose.orientation.y = float(s[4])
            person.pose.orientation.z = float(s[5])
            person.pose.orientation.w = float(s[6])
            people.append(person)

        db.commit()
        db.close()
        return people

    def update_locals(self, locals_input):
        locals_output = []

        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        for local in locals_input:
            pose_str = '{} {} {} {} {} {} {}'.format(
                local.pose.position.x,
                local.pose.position.y,
                local.pose.position.z,
                local.pose.orientation.x,
                local.pose.orientation.y,
                local.pose.orientation.z,
                local.pose.orientation.w)

            query = '''
                INSERT OR IGNORE INTO locals (name, pose) values
                ('{local_name}','{local_pose}');
                '''
            cursor.execute(query.format(
                local_name=local.name, local_pose=pose_str))

            query = '''
                UPDATE locals SET
                pose = '{local_pose}'
                WHERE name = '{local_name}';
                '''
            cursor.execute(query.format(
                local_name=local.name, local_pose=pose_str))

        query = '''
            SELECT name,pose FROM locals
            '''
        cursor.execute(query)
        locals_result = cursor.fetchall()
        locals_qtde = len(locals_result)
        locals_name = [row[0].encode("utf-8") for row in locals_result]
        locals_pose = [row[1].encode("utf-8") for row in locals_result]

        db.commit()
        db.close()

        for i in range(locals_qtde):
            s = locals_pose[i].split()
            local = Local()
            local.name = locals_name[i]
            local.pose.position.x = float(s[0])
            local.pose.position.y = float(s[1])
            local.pose.position.z = float(s[2])
            local.pose.orientation.x = float(s[3])
            local.pose.orientation.y = float(s[4])
            local.pose.orientation.z = float(s[5])
            local.pose.orientation.w = float(s[6])
            locals_output.append(local)
        return locals_output

    def update_objects(self, objects_input):
        objects_output = []

        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        for object in objects_input:
            pose_str = '{} {} {} {} {} {} {}'.format(
                object.pose.position.x,
                object.pose.position.y,
                object.pose.position.z,
                object.pose.orientation.x,
                object.pose.orientation.y,
                object.pose.orientation.z,
                object.pose.orientation.w)
            query = '''
                INSERT OR IGNORE INTO objects (name, pose, type) values
                ('{object_name}','{object_pose}','{object_type}');
                '''
            cursor.execute(query.format(
                object_name=object.name, object_pose=pose_str, object_type=object.type))
            query = '''
                UPDATE objects SET
                pose = '{object_pose}', type = '{object_type}'
                WHERE name = '{object_name}';
                '''
            cursor.execute(query.format(
                object_name=object.name, object_pose=pose_str, object_type=object.type))

        query = '''
            SELECT name,pose,type FROM objects
            '''
        cursor.execute(query)
        objects_result = cursor.fetchall()
        objects_qtde = len(objects_result)
        objects_name = [row[0].encode("utf-8") for row in objects_result]
        objects_pose = [row[1].encode("utf-8") for row in objects_result]
        objects_type = [row[2].encode("utf-8") for row in objects_result]

        db.commit()
        db.close()

        for i in range(objects_qtde):
            s = objects_pose[i].split()
            object = Object()
            object.name = objects_name[i]
            object.type = objects_type[i]
            object.pose.position.x = float(s[0])
            object.pose.position.y = float(s[1])
            object.pose.position.z = float(s[2])
            object.pose.orientation.x = float(s[3])
            object.pose.orientation.y = float(s[4])
            object.pose.orientation.z = float(s[5])
            object.pose.orientation.w = float(s[6])
            objects_output.append(object)
        return objects_output

    def update_people(self, people_input):
        people_output = []
        db = sqlite3.connect(self.db_path_output)
        cursor = db.cursor()

        for person in people_input:
            pose_str = '{} {} {} {} {} {} {}'.format(
                person.pose.position.x,
                person.pose.position.y,
                person.pose.position.z,
                person.pose.orientation.x,
                person.pose.orientation.y,
                person.pose.orientation.z,
                person.pose.orientation.w)
            pose_apr = get_approach_pose(person)
            pose_apr_str = '{} {} {} {} {} {} {}'.format(
                pose_apr.position.x,
                pose_apr.position.y,
                pose_apr.position.z,
                pose_apr.orientation.x,
                pose_apr.orientation.y,
                pose_apr.orientation.z,
                pose_apr.orientation.w)
            proxemic = get_proxemic()
            # person_interactions_str = " ".join(person.person_interactions)
            # object_interactions_str = " ".join(person.object_interactions)
            query = '''
                INSERT OR IGNORE INTO
                person (name, pose, pose_approach, proxemic)
                values ('{person_name}','{person_pose}', '{person_pose_apr}', '{person_proxemic}');
                '''
            cursor.execute(query.format(
                person_name=person.name,
                person_pose=pose_str,
                person_pose_apr=pose_apr_str,
                person_proxemic=proxemic))
            query = '''
                UPDATE person SET
                pose = '{person_pose}',
                pose_approach = '{person_pose_apr}',
                proxemic = '{person_proxemic}'
                WHERE name = '{person_name}';
                '''
            cursor.execute(query.format(
                person_name=person.name,
                person_pose=pose_str,
                person_pose_apr=pose_apr_str,
                person_proxemic=proxemic))

        query = '''
            SELECT name,pose,proxemic,pose_approach FROM person
            '''
        cursor.execute(query)
        people_result = cursor.fetchall()
        people_qtde = len(people_result)
        people_name = [row[0].encode("utf-8") for row in people_result]
        people_pose = [row[1].encode("utf-8") for row in people_result]
        people_proxemic = [row[2] for row in people_result]
        people_pose_approach = [row[3].encode("utf-8") for row in people_result]
        # people_person_interaction = [row[2].encode("utf-8") for row in people_result]
        # people_object_interaction = [row[3].encode("utf-8") for row in people_result]

        db.commit()
        db.close()

        for i in range(people_qtde):
            spo = people_pose[i].split()
            spoa = people_pose_approach[i].split()
            # pi = people_person_interaction[i].split()
            # po = people_object_interaction[i].split()
            person = Person()
            person.name = people_name[i]
            person.pose.position.x = float(spo[0])
            person.pose.position.y = float(spo[1])
            person.pose.position.z = float(spo[2])
            person.pose.orientation.x = float(spo[3])
            person.pose.orientation.y = float(spo[4])
            person.pose.orientation.z = float(spo[5])
            person.pose.orientation.w = float(spo[6])
            person.proxemic = people_proxemic[i]
            person.pose_approach.position.x = float(spoa[0])
            person.pose_approach.position.y = float(spoa[1])
            person.pose_approach.position.z = float(spoa[2])
            person.pose_approach.orientation.x = float(spoa[3])
            person.pose_approach.orientation.y = float(spoa[4])
            person.pose_approach.orientation.z = float(spoa[5])
            person.pose_approach.orientation.w = float(spoa[6])
            # person.person_interactions = pi
            # person.object_interactions = po
            people_output.append(person)

        return people_output


    # def get_pose_to_approach(self, person_pose):
    #     p = Pose()
    #     p.position.x = person_pose.position.x + 0.6
    #     p.position.y = person_pose.position.y + 0.6
    #     p.position.z = person_pose.position.z
    #     p.orientation.w = 1
    #     return p


# for person in self.leg_data:
#     pose_str = '{} {} {} {} {} {} {}'.format(
#         person.pos.x,
#         person.pos.y,
#         person.pos.z,
#         0,
#         0,
#         0,
#         1)
#     query = '''
#         INSERT OR IGNORE INTO person (name, pose) values
#         ('{person_name}','{person_pose}');
#         '''
#     cursor.execute(query.format(
#         person_name=person.name, person_pose=pose_str))
#     query = '''
#         UPDATE person SET
#         pose = '{person_pose}'
#         WHERE name = '{person_name}';
#         '''
#     cursor.execute(query.format(
#         person_name=person.name, person_pose=pose_str))
