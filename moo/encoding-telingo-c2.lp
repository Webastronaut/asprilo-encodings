% https://github.com/potassco/asprilo-encodings/tree/dd86e26a53e12d311e37eee8d7a8776579cb4b1f
% based on files of domain m (see ./m/encoding.ilp) from commit #dd86e26a53e12d311e37eee8d7a8776579cb4b1f

% telingo asprilo-encodings/m/encoding-telingo.lp asprilo-benchmark-instances/moo/structured/1x2x4/100sc/r02/x11_y6_n66_r2_s16_ps1_pr16_u16_o2_N001.lp 1 --stats -V0

% ./run-telingo-viz-output.sh m asprilo-benchmark-instances/moo/structured/1x2x4/100sc/r02/x11_y6_n66_r2_s16_ps1_pr16_u16_o2_N001.lp

% prefix m for "manhattan (distance)" according to the constraint
#program initial.
% --------------------------------------------------------------------------------
% REPRESENTATION

robot(R) :- init(object(robot,R),_).
shelf(S) :- init(object(shelf,S),_).
station(P) :- init(object(pickingStation,P),_).
product(A) :- init(object(product,A),_).

isRobot(robot(R)) :- robot(R).
isShelf(shelf(S)) :- shelf(S).
isStation(station(T)) :- station(T).
isProduct(product(A)) :- product(A).
isOrder(order(O)) :- order(O).

order(O) :- init(object(order,O),_).
ordered(order(O),product(A)) :- init(object(order,O), value(line,(A,_))). % IGNORING QUANTITIES
target(order(O),station(P)) :- init(object(order,O), value(pickingStation,P)).

shelved(shelf(S),product(A)) :- init(object(product,A), value(on,(S,_))). % IGNORING QUANTITIES

position((X,Y)) :- init(object(node,_), value(at,(X,Y))).

position_station(station(P),(X,Y)) :- init(object(pickingStation,P), value(at,(X,Y))).

position(robot(R),(X,Y)) :- init(object(robot,R), value(at,(X,Y))).
position(shelf(S),(X,Y)) :- init(object(shelf,S), value(at,(X,Y))).

highway(C) :- init(object(highway,_), value(at,C)).
% --------------------------------------------------------------------------------

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X0,Y0),(X+X0,Y+Y0)) :- position((X,Y)), direction((X0,Y0)), position((X+X0,Y+Y0)).

#program dynamic.
 { move(R,D) : _direction(D) } 1 :- _isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C) :- move(R,D), 'position(R,C0), _nextto(C0,D,C).
:- move(R,D), 'position(R,C), not _nextto(C,D,_).

% - inertia ---------------------------------------------------------------------
position(R,C) :- 'position(R,C), not move(R,_), _isRobot(R).

% - edge collision --------------------------------------------------------------
moveto(C0,C) :- _nextto(C0,D,C), 'position(R,C0), move(R,D).
:- moveto(C0,C), moveto(C,C0), C < C0.

% - vertex collision ------------------------------------------------------------
:- { position(R,C) : _isRobot(R) } > 1, _position(C).

% - auxiliaries -----------------------------------------------------------------
:- { position(R,C) } != 1, _isRobot(R). % REDUNDANT but PERFORMANT?

occurs(object(robot,R), action(move,D)) :- move(robot(R),D).

#show occurs/2.

#program final.

processed(A,R) :- _ordered(O,A), _shelved(S,A), _isRobot(R), _position(S,C), position(R,C).
:- _ordered(O,A), not processed(A,_).


% ################################################################################
% + CUSTOM CONSTRAINTS M
% ################################################################################

#program dynamic.
% helper preds
waits(R) :- _isRobot(R), not move(R).

% no running in circles or wobbling around
%% (2) eventually before, conjunction
:- position(R,C), _isRobot(R), &tel{ <? (position(R,C) & ~waits(R)) }.

% ################################################################################
% = CUSTOM CONSTRAINTS M
% ################################################################################