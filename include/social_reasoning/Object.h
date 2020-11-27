#include <string>
#include <geometry_msgs/Pose.h>

namespace social_reasoning
{
  class Object
  {
  private:
    std::string name;
    std::string type;
    geometry_msgs::Pose pose;

  public:
    Object(std::string _name, std::string _type, geometry_msgs::Pose _pose)
    {
      this->name = _name;
      this->type = _type;
      this->pose = _pose;
    }
    ~Object(){}

    std::string getName()
    {
      return this->name;
    }

    std::string getType()
    {
      return this->type;
    }

    geometry_msgs::Pose getPose()
    {
      return this->pose;
    }

    void setName(std::string _name)
    {
      this->name = _name;
    }

    void setType(std::string _type)
    {
      this->type = _type;
    }

    void setPosition(geometry_msgs::Pose _pose)
    {
      this->pose = _pose;
    }

    bool isName(const std::string& _name)
    {
      return this->name == _name;
    }

    std::string getMesh()
    {
      if(this->type == "tv")
        return "package://social_worlds/models/tv/model.dae";
      else if(this->type == "picture_frame")
        return "package://social_worlds/models/3dparty/picture_frame/meshes/frame.dae";
    }
  };
}
