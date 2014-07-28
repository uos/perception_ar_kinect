
:- module(knowrob_perception_ext,
    [
      create_object_perception_with_instance_check/4,
      create_action_inst_perception/7,
      same_object/3,
      get_pose/2
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
  % for now
  get_timepoint(Start), get_timepoint(End), 
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


%% get_pose
%
% extracts pose parameters from given Rotation Matrix

get_pose(RotM, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  owl_individual_of(RotM, knowrob:'RotationMatrix3D'),

  rdf_triple(knowrob:m00, RotM, RCC00),strip_literal_type(RCC00, RC00),atom_to_term(RC00,M00,_),
  rdf_triple(knowrob:m01, RotM, RCC01),strip_literal_type(RCC01, RC01),atom_to_term(RC01,M01,_),
  rdf_triple(knowrob:m02, RotM, RCC02),strip_literal_type(RCC02, RC02),atom_to_term(RC02,M02,_),
  rdf_triple(knowrob:m03, RotM, RCC03),strip_literal_type(RCC03, RC03),atom_to_term(RC03,M03,_),

  rdf_triple(knowrob:m10, RotM, RCC10),strip_literal_type(RCC10, RC10),atom_to_term(RC10,M10,_),
  rdf_triple(knowrob:m11, RotM, RCC11),strip_literal_type(RCC11, RC11),atom_to_term(RC11,M11,_),
  rdf_triple(knowrob:m12, RotM, RCC12),strip_literal_type(RCC12, RC12),atom_to_term(RC12,M12,_),
  rdf_triple(knowrob:m13, RotM, RCC13),strip_literal_type(RCC13, RC13),atom_to_term(RC13,M13,_),

  rdf_triple(knowrob:m20, RotM, RCC20),strip_literal_type(RCC20, RC20),atom_to_term(RC20,M20,_),
  rdf_triple(knowrob:m21, RotM, RCC21),strip_literal_type(RCC21, RC21),atom_to_term(RC21,M21,_),
  rdf_triple(knowrob:m22, RotM, RCC22),strip_literal_type(RCC22, RC22),atom_to_term(RC22,M22,_),
  rdf_triple(knowrob:m23, RotM, RCC23),strip_literal_type(RCC23, RC23),atom_to_term(RC23,M23,_),

  rdf_triple(knowrob:m30, RotM, RCC30),strip_literal_type(RCC30, RC30),atom_to_term(RC30,M30,_),
  rdf_triple(knowrob:m31, RotM, RCC31),strip_literal_type(RCC31, RC31),atom_to_term(RC31,M31,_),
  rdf_triple(knowrob:m32, RotM, RCC32),strip_literal_type(RCC32, RC32),atom_to_term(RC32,M32,_),
  rdf_triple(knowrob:m33, RotM, RCC33),strip_literal_type(RCC33, RC33),atom_to_term(RC33,M33,_).
