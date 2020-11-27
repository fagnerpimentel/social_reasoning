#include <string>
#include <geometry_msgs/Pose.h>

namespace social_reasoning
{
  class Person
  {
  private:
    std::string name;
    geometry_msgs::Pose pose;

  public:
    Person(std::string _name, geometry_msgs::Pose _pose)
    {
      this->name = _name;
      this->pose = _pose;
    }
    ~Person(){}

    std::string getName()
    {
      return this->name;
    }

    geometry_msgs::Pose getPose()
    {
      return this->pose;
    }

    void setName(std::string _name)
    {
      this->name = _name;
    }

    void setPose(geometry_msgs::Pose _pose)
    {
      this->pose = _pose;
    }

    bool isName(const std::string& _name)
    {
      return this->name == _name;
    }
  };
}
