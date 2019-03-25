% https://github.com/potassco/asprilo-encodings/tree/dd86e26a53e12d311e37eee8d7a8776579cb4b1f
% based on files of domain b (see ./abc/encoding-b.ilp) from commit #dd86e26a53e12d311e37eee8d7a8776579cb4b1f

% This is a simplification of domain A in sense that we completely ignore product quantities. With that, a shelf at a picking station can fill an order if it holds the required product.
% => one timestep per ordered product at the picking station (no matter the qty)

% telingo asprilo-encodings/b/encoding-telingo.lp asprilo-benchmark-instances/abc/structured/1x2x4/25sc/x11_y6_n66_r1_s4_ps1_pr8_u24_o2_N001.lp 1 --stats  --imax=100

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

% was in dynamic part
toShelf(S,C) :- shelved(S,A), ordered(O,A), target(O,P), position_station(P,C).

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
:- _highway(C), not carries(_,S), position(S,C), _isShelf(S).
:- _highway(C), waits(R), position(R,C).
% ################################################################################
% = control/highways.ilp
% ################################################################################



% ################################################################################
% + output-MPPD.ilp
% ################################################################################

occurs(object(robot,R),action(move,D)) :- move(robot(R),D).
occurs(object(robot,R),action(pickup,())) :- pickup(robot(R),_).
occurs(object(robot,R),action(putdown,())) :- putdown(robot(R),_).

occurs(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), toShelf(S,C), 'position(robot(R),C).
occurs(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), 'atShelf(S,C), 'carries(robot(R),S). % never occurs
occurs(object(robot,R),action(deliver,(O,A))) :- process(product(A),order(O),C), position(S,C), 'carries(robot(R),S), _shelved(S,product(A)).
occurs(object(robot,R),action(deliver,(O,A,N))) :- process(product(A),order(O),S,N), 'carries(robot(R),S).

#show occurs/2.
% ################################################################################
% = output-MPPD.ilp
% ################################################################################



% ################################################################################
% + goal-D-b.ilp
% ################################################################################
%#const rate=1.

{ process(A,O,C) : _ordered(O,A), _target(O,P), _position_station(P,C) } 1. %rate.

processed(A,O) :- process(A,O,C).
processed(A,O) :- 'processed(A,O).

:- process(A,O,C), 'processed(A,O). % what's been processed cannot be processed again

process(A,C) :- process(A,O,C).
process(C) :- process(A,C).

:- process(A,C), not 'position(S,C) : _shelved(S,A). % a product cannot be processed when its respective shelf is not on proper position

:- process(C), 'position(S,C), toShelf(S,C), not 'carries(_,S). % a shelf on correct position cannot be processed if it's not carried
:- process(C), 'position(R,C), _isRobot(R), not waits(R). % a process action can only occur if the robot on that position does wait

#program final.

:- _ordered(O,A), not processed(A,O).

:- carries(_,_).

:- _position_station(_,C), position(_,C).
% ################################################################################
% = goal-D-b.ilp
% ################################################################################



% ################################################################################
% + CUSTOM CONSTRAINTS B
% ################################################################################

#program initial.
% helper
contains_ordered_product(S,A) :- ordered(O,A), shelved(S,A).

#program dynamic.
% helper pred
article_delivered(robot(R),S,product(A)) :- process(product(A),order(O),C), position(S,C), 'carries(robot(R),S), _shelved(S,product(A)).

% no putdown between a pickup and a delivery
:- _contains_ordered_product(S,A), _isRobot(R), not &tel{ pickup(R,S) -> >? (~putdown(R,S) >? article_delivered(R,S,A)) }.

% only pickup if article is delivered afterwards
:- _contains_ordered_product(S,A), _isRobot(R), not &tel{ pickup(R,S) -> >? article_delivered(R,S,A) }.

% only pickup if not processed before
:- _contains_ordered_product(S,A), _isRobot(R0), pickup(R,S), &tel{ <? article_delivered(R0,S,A) }.

% shelves may not be picked up or put down more than once
:- _isRobot(R0), _isRobot(R1), _isShelf(S), &tel{ (pickup(R0,S) & < <? pickup(R1,S)) | (putdown(R0,S) & < <? putdown(R1,S)) }.

#program final.

% shelves must not be picked up unless they contain an ordered product
:- _isShelf(S), not _contains_ordered_product(S,_), _isRobot(R), &tel{ <? pickup(R,S) }.

% an agent not delivering anything may never pickup a shelf
:- _contains_ordered_product(S,A), _isRobot(R), &tel{ <* ~article_delivered(R,S,A) & <? (putdown(R,S) | pickup(R,S)) }.

% a shelf may never occupy the position of a picking station, which is not its target (domain b specific)
:- position_station(P,CP), toShelf(S,CS), CP!=CS, &tel{ <? position(S,CP) }.

% ################################################################################
% = CUSTOM CONSTRAINTS B
% ################################################################################