#include <json_prolog/prolog.h>

namespace knowrob_mapping
{
  std::string markerToObjClass(int id)
  {
    switch(id) {
      case 0: return "TetraPak";
      case 1: return "TetraPak";
      case 2: return "DrinkingGlass";
      case 3: return "DrinkingGlass";
      default: return "HumanScaleObject";}
  }


  std::string findObjInst(std::string& type, geometry_msgs::PoseStamped& pose)
  {
    tf::Quaternion q = tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Pose p;
    p.getBasis().setRotation(q);

    json_prolog::Prolog prolog;
    std::stringstream a;
    a << "same_object('http://ias.cs.tum.edu/kb/knowrob.owl#" << type << "', [" 
      <<static_cast<double>(p.getBasis().getRow(0).getX())<<","<< static_cast<double>(p.getBasis().getRow(0).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(0).getZ())<<","<< pose.pose.position.x<<","
      <<static_cast<double>(p.getBasis().getRow(1).getX())<<","<< static_cast<double>(p.getBasis().getRow(1).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(1).getZ())<<","<< pose.pose.position.y<<","
      <<static_cast<double>(p.getBasis().getRow(2).getX())<<","<< static_cast<double>(p.getBasis().getRow(2).getY())<<","
      <<static_cast<double>(p.getBasis().getRow(2).getZ())<<","
      <<pose.pose.position.z << ", 0.0, 0.0, 0.0, 1.0], ObjInst)";
    try {
      json_prolog::PrologBindings r = prolog.once(a.str());
      if(r.begin() != r.end()) {
        return r["ObjInst"];
      }
      else {
        return "none";
      }
    } catch (json_prolog::PrologQueryProxy::QueryError ex) {
      ROS_ERROR("[ar_action_listener]%s", ex.what());
      return "none";
    }
  }
}
