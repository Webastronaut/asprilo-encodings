#include <incmode>.

% clingo asprilo-encodings/m/encoding.lp asprilo-benchmark-instances/moo/structured/1x2x4/100sc/r02/x11_y6_n66_r2_s16_ps1_pr16_u16_o2_N002.lp 1 --stats -V0

% --------------------------------------------------------------------------------
% REPRESENTATION

robot(R)                     :- init(object(robot,R),          _).
shelf(S)                     :- init(object(shelf,S),          _).
station(P)                   :- init(object(pickingStation,P), _).
product(A)                   :- init(object(product,A),        _).

    isRobot(robot(R)) :- robot(R).
    isShelf(shelf(S)) :- shelf(S).
isStation(station(T)) :- station(T).
isProduct(product(A)) :- product(A).
    isOrder(order(O)) :- order(O).

  order(      O            ) :- init(object(order,O),          _).
ordered(order(O),product(A)) :- init(object(order,O),          value(line,(A,_))).      % IGNORING QUANTITIES
 target(order(O),station(P)) :- init(object(order,O),          value(pickingStation,P)).

shelved(shelf(S),product(A)) :- init(object(product,A),        value(on,(S,_))).        % IGNORING QUANTITIES

position(           (X,Y))   :- init(object(node,_),           value(at,(X,Y))).

position(station(P),(X,Y))   :- init(object(pickingStation,P), value(at,(X,Y))).

position(  robot(R),(X,Y),0) :- init(object(robot,R),          value(at,(X,Y))).
position(  shelf(S),(X,Y),0) :- init(object(shelf,S),          value(at,(X,Y))).

highway(            C    )   :- init(object(highway,_),        value(at,C)).
% --------------------------------------------------------------------------------

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')).

#program step(t).

{ move(R,D,t) : direction(D) } 1 :- isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C,t) :- move(R,D,t), position(R,C',t-1),     nextto(C',D,C).
                :- move(R,D,t), position(R,C ,t-1), not nextto(C ,D,_).

% - inertia ---------------------------------------------------------------------
position(R,C,t) :- position(R,C,t-1), not move(R,_,t), isRobot(R).

% - edge collision --------------------------------------------------------------
moveto(C',C,t) :- nextto(C',D,C), position(R,C',t-1), move(R,D,t).
 :- moveto(C',C,t), moveto(C,C',t), C < C'.

% - vertex collision ------------------------------------------------------------
 :- { position(R,C,t) : isRobot(R) }  > 1, position(C).

% - auxiliaries -----------------------------------------------------------------
 :- { position(R,C,t) } != 1, isRobot(R).             % REDUNDANT but PERFORMANT?

 #program check(t).

processed(A,R,t) :- ordered(O,A), shelved(S,A), isRobot(R), position(S,C,0),
                    position(R,C,t).

:- ordered(O,A), not processed(A,_,t), query(t).


#show.
%#show init/2.

#program check(t).

#show occurs(object(robot,R), action(move,D),     t) :    move(robot(R),D,t).
