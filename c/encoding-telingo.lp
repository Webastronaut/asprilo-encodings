% https://github.com/potassco/asprilo-encodings/tree/develop
% based on files of domain c (see ./abc/encoding-c.ilp) from commit #dd86e26a53e12d311e37eee8d7a8776579cb4b1f

% This is a simplification of domain B in the sense that for a robot at a picking station, all deliveries can be processed in parallel and within a single time step.

% telingo asprilo-encodings/c/encoding-telingo.lp asprilo-benchmark-instances/abc/structured/1x2x4/50sc/x11_y6_n66_r4_s8_ps1_pr48_u144_o8_N001.lp 1 --stats  --imax=35

% ################################################################################
% + input.lp
% ################################################################################
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
ordered(order(O),product(A)) :- init(object(order,O),value(line,(A,_))).      % IGNORING QUANTITIES
target(order(O),station(P)) :- init(object(order,O),value(pickingStation,P)).

shelved(shelf(S),product(A)) :- init(object(product,A),value(on,(S,_))).        % IGNORING QUANTITIES

position((X,Y)) :- init(object(node,_),value(at,(X,Y))).

position_station(station(P),(X,Y)) :- init(object(pickingStation,P),value(at,(X,Y))).

position(robot(R),(X,Y)) :- init(object(robot,R),value(at,(X,Y))).
position(shelf(S),(X,Y)) :- init(object(shelf,S),value(at,(X,Y))).

highway(C) :- init(object(highway,_),value(at,C)).
% ################################################################################
% = input.lp
% ################################################################################



% ################################################################################
% + action-MPP.ilp
% ################################################################################
direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')).

#program dynamic.

{ move(R,D) : _direction(D) ;
    pickup(R,S) : _isShelf(S) ;
    putdown(R,S) : _isShelf(S) } 1 :- _isRobot(R).
move(R) :- move(R,_).

waits(R) :- not pickup(R,_), not putdown(R,_), not move(R,_), _isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C) :- move(R,D), 'position(R,C'), _nextto(C',D,C).
:- move(R,D), 'position(R,C), not _nextto(C ,D,_).

% - pickup/3 --------------------------------------------------------------------
carries(R,S) :- pickup(R,S).
:- pickup(R,S), 'carries(R,_).
:- pickup(R,S), 'carries(_,S).
:- pickup(R,S), 'position(R,C), not 'position(S,C).
:- pickup(R,S), 'position(S,C), not 'position(R,C).

% - putdown/3 -------------------------------------------------------------------
:- putdown(R,S), not 'carries(R,S).

% - serves/4 --------------------------------------------------------------------
serves(R,S,P) :- position(R,C), carries(R,S), _position_station(P,C), strategy.

% - inertia ---------------------------------------------------------------------
position(R,C) :- 'position(R,C), not move(R,_), _isRobot(R).
carries(R,S) :- 'carries(R,S), not putdown(R,S).

% - (in)direct effects ----------------------------------------------------------
position(S,C) :- position(R,C), carries(R,S).
position(S,C) :- 'position(S,C), not carries(_,S), _isShelf(S).

% - edge collision --------------------------------------------------------------
moveto(C',C) :- _nextto(C',D,C), 'position(R,C'), move(R,D).
:- moveto(C',C), moveto(C,C'), C < C'.

% - vertex collision ------------------------------------------------------------
:- { position(R,C) : _isRobot(R) } > 1, _position(C).
:- { position(S,C) : _isShelf(S) } > 1, _position(C).

% - auxiliaries -----------------------------------------------------------------
:- { position(R,C) } != 1, _isRobot(R).
:- { position(S,C) } != 1, _isShelf(S).

:- { carries(R,S) } > 1, _isRobot(R).
:- { carries(R,S) } > 1, _isShelf(S).
% ################################################################################
% = action-MPP.ilp
% ################################################################################



% ################################################################################
% + control/highways.ilp
% ################################################################################
position(R,(X,Y)) :- positionX(R,X), positionY(R,Y).
:- _highway(C), not carries(_,S), position(S,C), _isShelf(S). % shelves must not stand on highways
:- _highway(C), waits(R), position(R,C). % waiting on a highway is not allowed
% ################################################################################
% = control/highways.ilp
% ################################################################################



% ################################################################################
% + output-MPPD.ilp
% ################################################################################
#show.
%#show init/2.

occurs(object(robot,R),action(move,D)) :- move(robot(R),D).
occurs(object(robot,R),action(pickup,())) :- pickup(robot(R),_).
occurs(object(robot,R),action(putdown,())) :- putdown(robot(R),_).

occurs_a(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), toShelf(S,C), 'position(robot(R),C).
occurs_b(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), 'atShelf(S,C), 'carries(robot(R),S).
occurs_c(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), position(S,C), 'carries(robot(R),S), _shelved(S,product(A)).
occurs_d(object(robot,R),action(deliver,(O,A,N))) :- process(product(A),order(O),S,N), 'carries(robot(R),S).

#show occurs/2.

% ################################################################################
% = output-MPPD.ilp
% ################################################################################



% ################################################################################
% + goal-D-b.ilp
% ################################################################################
instant(shelf(S)) :- _position(shelf(S),C), _position_station(_,C), strategy.

process(A,O,C) :- _ordered(O,A), _target(O,P), _position_station(P,C), _shelved(S,A), not _position(S,C), position(S,C).
process(A,O,C) :- _ordered(O,A), _target(O,P), _position_station(P,C), _shelved(S,A), _position(S,C), carries(_,S).
processed(A,C) :- process(A,O,C).
processed(A,C) :- 'processed(A,C).

#program final.

:- _ordered(O,A), _target(O,P), _position_station(P,C), not processed(A,C).

:- carries(_,_).

:- _position_station(_,C), position(_,C).
% ################################################################################
% = goal-D-b.ilp
% ################################################################################



% ################################################################################
% + CUSTOM CONSTRAINTS C
% ################################################################################

#program initial.
% helper
contains_ordered_product(S,A) :- ordered(O,A), shelved(S,A).

#program dynamic.

% helper pred
article_delivered(R,S,A) :- _ordered(O,A), _target(O,P), _position_station(P,C), _shelved(S,A), not _position(S,C), position(S,C), carries(R,S).

% no putdown between a pickup and a delivery
:- _contains_ordered_product(S,A), _isRobot(R), not &tel{ pickup(R,S) -> >? (~putdown(R,S) >? article_delivered(R,S,A)) }.

% only pickup if article is delivered afterwards
:- _contains_ordered_product(S,A), _isRobot(R), not &tel{ pickup(R,S) -> >? article_delivered(R,S,A) }.

% only pickup if not processed before
:- _contains_ordered_product(S,A), _isRobot(R0), pickup(R,S), &tel{ <? article_delivered(R0,S,A) }.

% shelves may not be picked up or put down more than once
:- _isRobot(R0), _isRobot(R1), _isShelf(S), &tel{ (pickup(R0,S) & < <? pickup(R1,S)) | (putdown(R0,S) & < <? putdown(R1,S)) }.

% waiting on picking station is not allowed (domain c)
:- _position_station(P,C), position(R,C), 'position(R,C), _isRobot(R).

#program final.

% shelves must not be picked up unless they contain an ordered product
:- _isShelf(S), not _contains_ordered_product(S,_), _isRobot(R), &tel{ <? pickup(R,S) }.

% an agent not delivering anything may never pick up or put down a shelf
:- _contains_ordered_product(S,A), _isRobot(R), &tel{ <* ~article_delivered(R,S,A) & <? (putdown(R,S) | pickup(R,S)) }.

% ################################################################################
% = CUSTOM CONSTRAINTS C
% ################################################################################