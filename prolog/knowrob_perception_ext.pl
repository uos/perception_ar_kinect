
:- module(knowrob_perception_ext,
    [
      create_object_perception_with_instance_check/5,
      create_action_inst_perception/7,
      same_object/3
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
  create_timepoint(StartTime, Start), create_timepoint(EndTime, End), 
  rdf_assert(ActionInst, knowrob:'startTime', Start),
  rdf_assert(ActionInst, knowrob:'endTime', End).



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

create_object_perception_with_instance_check(ObjClass, ObjPose, PerceptionTypes, TimeStamp, ObjInst) :-
  same_object(ObjClass, ObjPose, ObjInst) ->
  % if already existing
  create_instance_perception_with_time(ObjInst, ObjPose, PerceptionTypes, TimeStamp);
  % else create new object
  create_object_perception_with_time(ObjClass, ObjPose, PerceptionTypes, TimeStamp, ObjInst).


%% similar_object
%
% checks wether an object instance of ObjClass, located close to the given ObjPose
% already exists in the database at the current time
%
% @param ObjClass  Class of the object instance
% @param ObjPose   Pose where the object should be located
% @param ObjInst   found object instance

same_object(ObjClass, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], ObjInst) :-
  get_timepoint(CurrentTime),
  owl_individual_of(ObjInst, ObjClass),
  % find latest detection of the Object that is valid at CurrentTime
  object_detection(ObjInst, CurrentTime, Detection),
  rdf_triple(knowrob:eventOccursAt, Detection, DMatrix),
  rdf_triple(knowrob:m03, DMatrix, DCxx),strip_literal_type(DCxx, DCx),atom_to_term(DCx,DX,_),
  rdf_triple(knowrob:m13, DMatrix, DCyy),strip_literal_type(DCyy, DCy),atom_to_term(DCy,DY,_),
  rdf_triple(knowrob:m23, DMatrix, DCzz),strip_literal_type(DCzz, DCz),atom_to_term(DCz,DZ,_),
  % check if distance is below 5cm
  % TODO: adapt distance threshold to time passed between two views
  =<( (((DX-M03)*(DX-M03))+((DY-M13)*(DY-M13))+((DZ-M23)*(DZ-M23))), 0.25).



%% create_instance_perception_with_time
%
% creates a perception instance and the pose matrix where the ObjInst was perceived
% and links them to the given ObjInst

create_instance_perception_with_time(ObjInst, ObjPose, PerceptionTypes, TimeStamp) :-
   create_perception_instance_with_time(PerceptionTypes, Perception, TimeStamp),
   set_object_perception(ObjInst, Perception),
   set_perception_pose(Perception, ObjPose).


%% create_object_perception_with_time
%
% creates a new object, a perception instance and the pose matrix where the ObjInst was perceived
% and links them all together 

create_object_perception_with_time(ObjClass, ObjPose, PerceptionTypes, TimeStamp, ObjInst) :-
    rdf_instance_from_class(ObjClass, ObjInst),
    create_perception_instance_with_time(PerceptionTypes, Perception, TimeStamp),
    set_object_perception(ObjInst, Perception),
    set_perception_pose(Perception, ObjPose).



%% create_perception_instance_with_time
%
% create_perception_instance having all the types in PerceptionTypes and
% the given TimeStamp

create_perception_instance_with_time(PerceptionTypes, Perception, TimeStamp) :-
  % create individual from first type in the list
  nth0(0, PerceptionTypes, PType),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PType, PClass),
  rdf_instance_from_class(PClass, Perception),
  % set all other types
  findall(PC, (member(PT, PerceptionTypes),
               atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PT, PC),
               rdf_assert(Perception, rdf:type, PC)), _),
  % create detection time point
  create_timepoint(TimeStamp, TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).



