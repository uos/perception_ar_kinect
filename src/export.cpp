
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>


using namespace std;
using namespace json_prolog;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "export");

    Prolog pl;

    string file = "/home/student/a/ahaeuser/ros-hydro/dry/stacks/perception_ar_kinect/owl_export/";
    stringstream b;
    b << "export_object_class(knowrob:'DrinkingGlass_bneXbLGX', '" << file << "drinkingGlass.owl'),"
      << "export_object_class(knowrob:'TetraPak_vUXiHMJy', '" << file << "tetraPak.owl'),"
      << "owl_individual_of(F, knowrob:'FillingProcess'),"
      << "export_object_class(F, '" << file << "fillingProcess.owl')";
    std::cout << b.str() << endl;
    pl.query(b.str());
}
