
:- module(import_owl,
    [
      import_owl_files/1
    ]).

:- use_module(library('owl_export')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).

:- rdf_meta
   import_owl_files(r).

%% import_export_owl
%
import_owl_files([]).

import_owl_files(List):-
  append([Head], Tail, List),
  owl_parse(Head , false, false, true),
  import_owl_files(Tail).
