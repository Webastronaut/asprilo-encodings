#include <incmode>.

% https://github.com/potassco/asprilo-encodings/tree/develop
% based on files of domain c (see ./abc/encoding-c.ilp) from commit #dd86e26a53e12d311e37eee8d7a8776579cb4b1f

% clingo asprilo-encodings/c/encoding.lp asprilo-benchmark-instances/abc/structured/1x2x4/25sc/x11_y6_n66_r1_s4_ps1_pr8_u24_o2_N001.lp 1 --stats -V0

% ################################################################################
% + input.lp
% ################################################################################
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
ordered(order(O),product(A)) :- init(object(order,O),value(line,(A,_))).      % IGNORING QUANTITIES
target(order(O),station(P)) :- init(object(order,O),value(pickingStation,P)).

shelved(shelf(S),product(A)) :- init(object(product,A),value(on,(S,_))).        % IGNORING QUANTITIES

position((X,Y)) :- init(object(node,_),value(at,(X,Y))).

position(station(P),(X,Y)) :- init(object(pickingStation,P),value(at,(X,Y))).

position(robot(R),(X,Y),0) :- init(object(robot,R),value(at,(X,Y))).
position(shelf(S),(X,Y),0) :- init(object(shelf,S),value(at,(X,Y))).

highway(C) :- init(object(highway,_),value(at,C)).
% ################################################################################
% = input.lp
% ################################################################################



% ################################################################################
% + action-MPP.ilp
% ################################################################################
direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')).

#program step(t).

{ move(R,D,t) : direction(D) ;
    pickup(R,S,t) : isShelf(S) ;
    putdown(R,S,t) : isShelf(S) } 1 :- isRobot(R).

waits(R,t) :- not pickup(R,_,t), not putdown(R,_,t), not move(R,_,t), isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C,t) :- move(R,D,t), position(R,C',t-1), nextto(C',D,C).
:- move(R,D,t), position(R,C ,t-1), not nextto(C ,D,_).

% - pickup/3 --------------------------------------------------------------------
carries(R,S,t) :- pickup(R,S,t).
:- pickup(R,S,t), carries(R,_,t-1).
:- pickup(R,S,t), carries(_,S,t-1).
:- pickup(R,S,t), position(R,C,t-1), not position(S,C,t-1).
:- pickup(R,S,t), position(S,C,t-1), not position(R,C,t-1).

% - putdown/3 -------------------------------------------------------------------
:- putdown(R,S,t), not carries(R,S,t-1).

% - serves/4 --------------------------------------------------------------------
serves(R,S,P,t) :- position(R,C,t), carries(R,S,t), position(P,C), strategy.

% - inertia ---------------------------------------------------------------------
position(R,C,t) :- position(R,C,t-1), not move(R,_,t), isRobot(R).
carries(R,S,t) :-  carries(R,S,t-1), not putdown(R,S,t).

% - (in)direct effects ----------------------------------------------------------
position(S,C,t) :- position(R,C,t), carries(R,S,t).
position(S,C,t) :- position(S,C,t-1), not carries(_,S,t), isShelf(S).

% - edge collision --------------------------------------------------------------
moveto(C',C,t) :- nextto(C',D,C), position(R,C',t-1), move(R,D,t).
:- moveto(C',C,t), moveto(C,C',t), C < C'.

% - vertex collision ------------------------------------------------------------
:- { position(R,C,t) : isRobot(R) } > 1, position(C).
:- { position(S,C,t) : isShelf(S) } > 1, position(C).

% - auxiliaries -----------------------------------------------------------------
:- { position(R,C,t) } != 1, isRobot(R).
:- { position(S,C,t) } != 1, isShelf(S).

:- { carries(R,S,t) } > 1, isRobot(R).
:- { carries(R,S,t) } > 1, isShelf(S).
% ################################################################################
% = action-MPP.ilp
% ################################################################################



% ################################################################################
% + goal-D-c.ilp
% ################################################################################
instant(shelf(S)) :- position(shelf(S),C,0), position(_,C), strategy.

#program step(t).

process(A,O,C,t) :- ordered(O,A), target(O,P), position(P,C), shelved(S,A), not position(S,C,0),
                    position(S,C,t).
process(A,O,C,t) :- ordered(O,A), target(O,P), position(P,C), shelved(S,A), position(S,C,0),
                    carries(_,S,t).
processed(A,C,t) :- process(A,O,C,t).
processed(A,C,t) :- processed(A,C,t-1).

#program check(t).

:- ordered(O,A), target(O,P), position(P,C), not processed(A,C,t), query(t).

:- carries(_,_,t), query(t).

:- position(_,C), position(_,C,t), query(t).
% ################################################################################
% = goal-D-b.ilp
% ################################################################################



% ################################################################################
% + control/highways.ilp
% ################################################################################
#program step(t).

position(R,(X,Y),t) :- positionX(R,X,t), positionY(R,Y,t).
:- highway(C), not carries(_,S,t), position(S,C,t), isShelf(S).
:- highway(C), waits(R,t), position(R,C,t).
% ################################################################################
% = control/highways.ilp
% ################################################################################



% ################################################################################
% + output-MPPD.ilp
% ################################################################################
#show.
%#show init/2.

#program step(t).

occurs(object(robot,R),action(move,D),t) :- move(robot(R),D,t).
occurs(object(robot,R),action(pickup,()),t) :- pickup(robot(R),_,t).
occurs(object(robot,R),action(putdown,()),t) :- putdown(robot(R),_,t).

occurs(object(robot,R),action(deliver,(O,A)),t) :- process(product(A),order(O),C,t), toShelf(S,C), position(robot(R),C,t-1).
occurs(object(robot,R),action(deliver,(O,A)),t) :- process(product(A),order(O),C,t), atShelf(S,C,t-1), carries(robot(R),S,t-1).
occurs(object(robot,R),action(deliver,(O,A)),t) :- process(product(A),order(O),C,t), position(S,C,t), carries(robot(R),S,t-1), shelved(S,product(A)).
occurs(object(robot,R),action(deliver,(O,A,N)),t) :- process(product(A),order(O),S,N,t), carries(robot(R),S,t-1).

#show occurs/3.
% ################################################################################
% = output-MPPD.ilp
% ################################################################################