#include "ros/ros.h"

#include "social_msgs/People.h"
#include "social_msgs/Objects.h"
#include "social_msgs/PointArray.h"

#include <geometry_msgs/Point.h>

#include "social_reasoning/Person.h"
#include "social_reasoning/Object.h"

#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>
#include <algorithm>

std::vector<social_reasoning::Person> people;
std::vector<social_reasoning::Object> objects;

void peopleCallback(const social_msgs::People::ConstPtr& msg)
{
  // ROS_INFO("people %i", (int)msg->people.size());
  for (size_t i = 0; i < msg->people.size(); i++) {
    std::string name = msg->people[i].name;
    geometry_msgs::Pose pose = msg->people[i].pose;

    std::vector<social_reasoning::Person>::iterator it = find_if(people.begin(), people.end(),
      boost::bind(&social_reasoning::Person::isName, boost::placeholders::_1, name));

    if (it != people.end())
    {
      (*it).setPose(pose);
    }
    else
    {
      social_reasoning::Person person(name, pose);
      people.push_back(person);
    }
  }

}

void objectsCallback(const social_msgs::Objects::ConstPtr& msg)
{
  // ROS_INFO("objects %i", (int)msg->objects.size());
  for (size_t i = 0; i < msg->objects.size(); i++) {
    std::string name = msg->objects[i].name;
    std::string type = msg->objects[i].type;
    geometry_msgs::Pose pose = msg->objects[i].pose;

    std::vector<social_reasoning::Object>::iterator it = find_if(objects.begin(), objects.end(),
      boost::bind(&social_reasoning::Object::isName, boost::placeholders::_1, name));

    if (it != objects.end())
    {
      (*it).setPosition(pose);
    }
    else
    {
      social_reasoning::Object object(name, type, pose);
      objects.push_back(object);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "social_reasoning_node");
  ros::NodeHandle n;

  ros::Rate r(100);

  ros::Subscriber sub_people = n.subscribe("people", 1000, peopleCallback);
  ros::Subscriber sub_objects = n.subscribe("objects", 1000, objectsCallback);

  ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("social_marker", 1);

  ros::Publisher pub_layer_points_people = n.advertise<social_msgs::PointArray>("points_people", 1);
  ros::Publisher pub_layer_points_objects = n.advertise<social_msgs::PointArray>("points_objects", 1);
  ros::Publisher pub_layer_points_interaction = n.advertise<social_msgs::PointArray>("points_interaction", 1);

  double interaction_distance = 2.5;
  double interaction_point_distance = 0.1;

  while (ros::ok())
  {
    social_msgs::PointArray layer_points_people;
    social_msgs::PointArray layer_points_objects;
    social_msgs::PointArray layer_points_interaction;


    for (size_t i = 0; i < people.size(); i++) {

      visualization_msgs::Marker marker_body;
      // Set the frame ID and timestamp.
      marker_body.header.frame_id = "/map";
      marker_body.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.
      marker_body.ns = "people_body";
      marker_body.id = i;
      // Set the marker type.
      marker_body.type = visualization_msgs::Marker::CYLINDER;
      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker_body.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.
      marker_body.pose = people[i].getPose();
      marker_body.pose.position.z = marker_body.pose.position.z - 0.3;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_body.scale.x = 0.2; // width
      marker_body.scale.y = 1.0; // height
      marker_body.scale.z = 0.2; // depth
      // Set the color -- be sure to set alpha to something non-zero!
      marker_body.color.r = 1.0f;
      marker_body.color.g = 0.0f;
      marker_body.color.b = 0.0f;
      marker_body.color.a = 1.0;
      // lifetime
      marker_body.lifetime = ros::Duration();

      visualization_msgs::Marker marker_head;
      // Set the frame ID and timestamp.
      marker_head.header.frame_id = "/map";
      marker_head.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.
      marker_head.ns = "people_head";
      marker_head.id = i;
      // Set the marker type.
      marker_head.type = visualization_msgs::Marker::SPHERE;
      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker_head.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.
      marker_head.pose = people[i].getPose();
      marker_head.pose.position.z = marker_head.pose.position.z + 0.4;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_head.scale.x = 0.2;
      marker_head.scale.y = 0.2;
      marker_head.scale.z = 0.2;
      // Set the color -- be sure to set alpha to something non-zero!
      marker_head.color.r = 1.0f;
      marker_head.color.g = 0.0f;
      marker_head.color.b = 0.0f;
      marker_head.color.a = 1.0;
      // lifetime
      marker_head.lifetime = ros::Duration();

      // publish
      pub_marker.publish(marker_body);
      pub_marker.publish(marker_head);

      geometry_msgs::Point p_people;
      p_people = people[i].getPose().position;
      layer_points_people.points.push_back(p_people);

    }

    for (size_t i = 0; i < objects.size(); i++) {
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.      layer_points_interaction.points.push_back(p_interacao);

      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.
      marker.ns = "objects";
      marker.id = i;
      // Set the marker type.
      // marker.type = visualization_msgs::Marker::SPHERE;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = objects[i].getMesh();
      marker.mesh_use_embedded_materials = true;
      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.
      marker.pose = objects[i].getPose();
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      // Set the color -- be sure to set alpha to something non-zero!
      // marker.color.r = 1.0f;
      // marker.color.g = 1.0f;
      // marker.color.b = 0.0f;
      // marker.color.a = 1.0;
      // lifetime
      marker.lifetime = ros::Duration();
      pub_marker.publish(marker);

      geometry_msgs::Point p_object;
      p_object = objects[i].getPose().position;
      layer_points_objects.points.push_back(p_object);
    }

    int i = 0;
    for (size_t i_o = 0; i_o < objects.size(); i_o++) {
    for (size_t i_p = 0; i_p < people.size(); i_p++) {

      geometry_msgs::Point p1 = people[i_p].getPose().position;
      geometry_msgs::Point p2 = objects[i_o].getPose().position;

      double dist = sqrt(pow(p2.x-p1.x,2) + pow(p2.y-p1.y,2));
      double ang = atan2(p2.y-p1.y, p2.x-p1.x);

      if(dist > interaction_distance) continue;
      i++;

      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.
      marker.ns = "interaction_object";
      marker.id = i;
      // Set the marker type.
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      // lifetime
      marker.lifetime = ros::Duration();


      double cont = 0;
      while (cont < dist){
        double dx = cont * cos(ang);
        double dy = cont * sin(ang);
        cont += interaction_point_distance;

        // create point
        geometry_msgs::Point p_interacao;
        p_interacao.x = p1.x + dx;
        p_interacao.y = p1.y + dy;
        p_interacao.z = (p1.z + p2.z)/2;
        // Set the points of the marker.
        marker.points.push_back(p_interacao);
        layer_points_interaction.points.push_back(p_interacao);
      }

      pub_marker.publish(marker);
    }}


    int ii = 0;
    for (size_t i_p1 = 0; i_p1 < people.size(); i_p1++) {
    for (size_t i_p2 = 0; i_p2 < people.size(); i_p2++) {

      geometry_msgs::Point p1 = people[i_p1].getPose().position;
      geometry_msgs::Point p2 = people[i_p2].getPose().position;

      double dist = sqrt(pow(p2.x-p1.x,2) + pow(p2.y-p1.y,2));
      double ang = atan2(p2.y-p1.y, p2.x-p1.x);

      if(dist > interaction_distance) continue;
      ii++;

      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      // Set the namespace and id for this marker.
      marker.ns = "interaction_people";
      marker.id = ii;
      // Set the marker type.
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      // lifetime
      marker.lifetime = ros::Duration();

      double cont = 0;
      while (cont < dist){
        double dx = cont * cos(ang);
        double dy = cont * sin(ang);
        cont += interaction_point_distance;

        // create point
        geometry_msgs::Point p_interacao;
        p_interacao.x = p1.x + dx;
        p_interacao.y = p1.y + dy;
        p_interacao.z = (p1.z + p2.z)/2;
        // Set the points of the marker.
        marker.points.push_back(p_interacao);
        layer_points_interaction.points.push_back(p_interacao);
      }

      pub_marker.publish(marker);
    }}

    pub_layer_points_people.publish(layer_points_people);
    pub_layer_points_objects.publish(layer_points_objects);
    pub_layer_points_interaction.publish(layer_points_interaction);

    ros::spinOnce();
    // r.sleep();
  }
  // ros::spin();

  return 0;
}
