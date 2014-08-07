#include <json_prolog/prolog.h>

namespace knowrob_mapping
{
  /*
   * maps marker ids to corresponding knowrob classes
   */
  std::string markerToObjClass(int id)
  {
    switch(id) {
      case 0: return "TetraPak";
      case 1: return "TetraPak";
      case 2: return "DrinkingGlass";
      case 3: return "DrinkingGlass";
      default: return "HumanScaleObject";}
  }

  /*
   * identifies object instance inside knowrob corresponding
   * to object class observed at a certain position
   *
   * @param type     Object Class
   * @param pose     Pose of object in Map Frame
   * @param objInst  found corresponding Object Instance
   */

  bool findObjInst(std::string& type, geometry_msgs::PoseStamped& pose, std::string& objInst)
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
      json_prolog::PrologQueryProxy r = prolog.query(a.str());
      json_prolog::PrologQueryProxy::iterator it = r.begin();
      if(it != r.end()) {
        json_prolog::PrologBindings te = *it;
        objInst = te["ObjInst"].toString();
        return true;
      }
      else {
        return false;
      }
    } catch (json_prolog::PrologQueryProxy::QueryError ex) {
      ROS_ERROR("[knowrob_mapping]%s", ex.what());
      return false;
    }
  }
}
