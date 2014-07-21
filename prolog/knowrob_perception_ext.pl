
:- module(knowrob_perception_ext,
    [
      create_object_perception_with_instance_check/4,
      create_action_inst_perception/7
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_objects')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).


:- rdf_meta
   create_object_perception_with_instance_check(r, +, +, -).
   create_action_inst_perception(r, +, +, +, +, +, -).



%% create_action_inst_perception
%
% creates an ActionInst with the given ObjActOns, To- and FromLocations
%
% @param Action       Type of Action to be created
% @param ObjActOnSet  List of Objects/ObjectInstances to be objectsActedOn
% @param ToLocSet     List of Locations/LocationInstances to be toLocations
% @param FromLocSet   List of Locations/LocationInstances to be fromLocations
% @param StartTime    Time the action started
% @param EndTime      Time the action ended
% @param ActionInst   created ActionInstance 

create_action_inst_perception(Action, ObjActOnSet, ToLocSet, FromLocSet, StartTime, EndTime, ActionInst) :-
  rdf_instance_from_class(Action, ActionInst),
  forall(member(ObjType, ObjActOnSet),
    ((owl_individual_of(ObjType, owl:'Class') ->
      rdf_instance_from_class(ObjType, ObjActOn);
      (ObjActOn = ObjType)),
    rdf_assert(ActionInst, knowrob:'objectActedOn', ObjActOn))),
  forall(member(ToLocType, ToLocSet),
    ((owl_individual_of(ToLocType, owl:'Class') ->
      rdf_instance_from_class(ToLocType, ToLoc);
      (ToLoc = ToLocType)),
    rdf_assert(ActionInst, knowrob:'toLocation', ToLoc))),
  forall(member(FromLocType, FromLocSet),
    ((owl_individual_of(FromLocType, owl:'Class') ->
      rdf_instance_from_class(FromLocType, FromLoc);
      (FromLoc = FromLocType)),
    rdf_assert(ActionInst, knowrob:'fromLocation', FromLoc))),
  rdf_assert(ActionInst, knowrob:'startTime', StartTime),
  rdf_assert(ActionInst, knowrob:'endTime', EndTime).



%% create_object_perception_with_instance_check
%
% creates a perception instance including the pose matrix where the object was perceived
% and links it to the object instance 
% if an object of the same class at a similar location already exists, a new perception instance
% is created for such, otherwise a new object instance is created
%
% @param ObjClass         Class of the perceived object
% @param ObjPose          Pose of the perceived object
% @param PerceptionTypes  PerceptionType used (eg. ARKinectObjectPerception)
% @param ObjInst          created or linked object instance

create_object_perception_with_instance_check(ObjClass, ObjPose, PerceptionTypes, ObjInst) :-
  same_object(ObjClass, ObjPose, ObjInst) ->
  % if already existing
  create_instance_perception(ObjInst, ObjPose, PerceptionTypes);
  % else create new object
  create_object_perception(ObjClass, ObjPose, PerceptionTypes, ObjInst).


%% similar_object
%
% checks wether an object instance of ObjClass, located close to the given ObjPose
% already exists in the database at the current time
%
% @param ObjClass  Class of the object instance
% @param ObjPose   Pose where the object should be located
% @param ObjInst   found object instance

same_object(ObjClass, ObjPose, ObjInst) :-
  get_timepoint(CurrentTime),
  owl_individual_of(ObjInst, ObjClass),
  % find all detections of the Object that are valid at CurrentTime
  object_detection(ObjInst, CurrentTime, Detection),
  rdf_triple(knowrob:eventOccursAt, Detection, DMatrix),
  rdf_triple(knowrob:m03, DMatrix, DCxx),strip_literal_type(DCxx, DCx),atom_to_term(DCx,DX,_),
  rdf_triple(knowrob:m13, DMatrix, DCyy),strip_literal_type(DCyy, DCy),atom_to_term(DCy,DY,_),
  rdf_triple(knowrob:m23, DMatrix, DCzz),strip_literal_type(DCzz, DCz),atom_to_term(DCz,DZ,_),
  % without cut deadlock-error for ObjClass = knowrob:TetraPak,
  % however works for e.g. DrinkingGlass, Orange_Juice without cut
  !,
  create_pose(ObjPose, Loc),
  rdf_triple(knowrob:m03, Loc, LCxx),strip_literal_type(LCxx, LCx),atom_to_term(LCx,LX,_),
  rdf_triple(knowrob:m13, Loc, LCyy),strip_literal_type(LCyy, LCy),atom_to_term(LCy,LY,_),
  rdf_triple(knowrob:m23, Loc, LCzz),strip_literal_type(LCzz, LCz),atom_to_term(LCz,LZ,_),

  % check if distance is below 5cm
  =<( (((DX-LX)*(DX-LX))+((DY-LY)*(DY-LY))+((DZ-LZ)*(DZ-LZ))), 0.25).



%% create_instance_perception
%
% creates a perception instance and the pose matrix where the ObjInst was perceived
% and links them to the given ObjInst

create_instance_perception(ObjInst, ObjPose, PerceptionTypes) :-
   create_perception_instance(PerceptionTypes, Perception),
   set_object_perception(ObjInst, Perception),
   set_perception_pose(Perception, ObjPose).

