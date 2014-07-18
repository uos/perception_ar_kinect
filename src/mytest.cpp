
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>


using namespace std;
using namespace json_prolog;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_test");

    Prolog pl;

    string a = "owl_individual_of(ObjInst, 'http://ias.cs.tum.edu/kb/knowrob.owl#HumanScaleObject')";
    PrologQueryProxy test = pl.query(a);
    cout << "HumanScaleObjects: " << endl;
    for(PrologQueryProxy::iterator it=test.begin(); it != test.end(); it++)
    {
      PrologBindings te = *it;
      cout << te["ObjInst"] << endl;
    }
    cout << endl;

    a = "owl_individual_of(A, knowrob:'DrinkingGlass'), owl_has(X, knowrob:objectActedOn, A),";
    a += "owl_has(X, knowrob:eventOccursAt, B)";
    test = pl.query(a);
    cout << "Rotations: ";
    for(PrologQueryProxy::iterator it=test.begin(); it != test.end(); it++)
    {
      PrologBindings te = *it;
      cout << te["B"] << endl;
    }
    cout << endl;
    
    a = "owl_individual_of(A, knowrob:'DrinkingGlass'), aggregate_all(count, owl_has(X, knowrob:objectActedOn, A),Count)";
    test = pl.query(a);
    cout << "DrinkingGlass Perceptions: ";
    for(PrologQueryProxy::iterator it=test.begin(); it != test.end(); it++)
    {
      PrologBindings te = *it;
      cout << te["Count"] << endl;
    }
    cout << endl;

    a = "owl_individual_of(A, knowrob:'TetraPak'), aggregate_all(count, owl_has(X, knowrob:objectActedOn, A),Count)";
    test = pl.query(a);
    cout << "TetraPak Perceptions: ";
    for(PrologQueryProxy::iterator it=test.begin(); it != test.end(); it++)
    {
      PrologBindings te = *it;
      cout << te["Count"] << endl; 
    }
    cout << endl;
}


