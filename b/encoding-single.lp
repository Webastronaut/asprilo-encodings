% https://github.com/potassco/asprilo-encodings/tree/develop
% based on commit #dd86e26a53e12d311e37eee8d7a8776579cb4b1f

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

%#const horizon=1.
time(1..horizon).

direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')).



 {    move(R,D,T) : direction(D) ;
    pickup(R,S,T) : isShelf(S)   ;
   putdown(R,S,T) : isShelf(S)   } 1 :- isRobot(R), time(T).

waits(R,T) :- not pickup(R,_,T), not putdown(R,_,T), not move(R,_,T), isRobot(R), time(T).

% - move/3 ----------------------------------------------------------------------
position(R,C,T) :- move(R,D,T),    position(R,C',T-1),     nextto(C',D,C).
                :- move(R,D,T),    position(R,C ,T-1), not nextto(C, D,_).

% - pickup/3 --------------------------------------------------------------------
 carries(R,S,T) :- pickup(R,S,T).
                :- pickup(R,S,T),   carries(R,_,T-1).
                :- pickup(R,S,T),   carries(_,S,T-1).                                                    % NEEDED (position of two robots)!?
                :- pickup(R,S,T),  position(R,C,T-1), not position(S,C,T-1).
                :- pickup(R,S,T),  position(S,C,T-1), not position(R,C,T-1).

% - putdown/3 -------------------------------------------------------------------
                :- putdown(R,S,T), not carries(R,S,T-1).

% - serves/4 --------------------------------------------------------------------
serves(R,S,P,T) :- position(R,C,T), carries(R,S,T), position(P,C), strategy.

% - inertia ---------------------------------------------------------------------
position(R,C,T) :- position(R,C,T-1), not    move(R,_,T), isRobot(R), time(T).
 carries(R,S,T) :-  carries(R,S,T-1), not putdown(R,S,T),             time(T).

% - (in)direct effects ----------------------------------------------------------
position(S,C,T) :- position(R,C,T  ),     carries(R,S,T).
position(S,C,T) :- position(S,C,T-1), not carries(_,S,T), isShelf(S), time(T).

% - edge collision --------------------------------------------------------------
moveto(C',C,T) :- nextto(C',D,C), position(R,C',T-1), move(R,D,T).
 :- moveto(C',C,T), moveto(C,C',T), C < C'.

% - vertex collision ------------------------------------------------------------
 :- { position(R,C,T) : isRobot(R) }  > 1, position(C), time(T).
 :- { position(S,C,T) : isShelf(S) }  > 1, position(C), time(T).

% - auxiliaries -----------------------------------------------------------------
 :- { position(R,C,T) } != 1, isRobot(R), time(T).
 :- { position(S,C,T) } != 1, isShelf(S), time(T).

 :- { carries(R,S,T) } > 1, isRobot(R), time(T).
 :- { carries(R,S,T) } > 1, isShelf(S), time(T).

 #const rate=1.

toShelf(S,C) :- shelved(S,A), ordered(O,A), target(O,P), position(P,C).



 { process(A,O,C,T) : ordered(O,A), target(O,P), position(P,C) } rate :- time(T).

processed(A,O,T) :- process(A,O,C,T).
processed(A,O,T) :- processed(A,O,T-1), time(T).

:- process(A,O,C,T), processed(A,O,T-1).



process(A,C,T) :- process(A,O,C,T).

:- process(A,C,T), not position(S,C,T-1) : shelved(S,A).

process(C,T) :- process(A,C,T).

:- process(C,T), position(S,C,T-1), toShelf(S,C), not carries(_,S,T-1).
:- process(C,T), position(R,C,T-1), isRobot(R), not waits(R,T).



:- ordered(O,A), not processed(A,O,horizon).

:- carries(_,_,horizon).

:- position(_,C), position(_,C,horizon).

% lexorder(A,O1,O2) :- ordered(O1,A), target(O1,P1), position(P1,C),
%                      ordered(O2,A), target(O2,P2), position(P2,C), O1 < O2,
%                      O <= O1 : ordered(O,A), target(O,P), position(P,C), O < O2.

% :- lexorder(A,O1,O2), process(A,O2,C,T), rate = 1, not processed(A,O1,T-1).
% :- lexorder(A,O1,O2), process(A,O2,C,T), rate > 1, not processed(A,O1,T).

position(R,(X,Y),T) :- positionX(R,X,T), positionY(R,Y,T).
:- highway(C), not carries(_,S,T), position(S,C,T), isShelf(S).
:- highway(C),         waits(R,T), position(R,C,T).

#show.
#show init/2.



#show occurs(object(robot,R), action(move,   D      ), T) :    move(robot(R),D,T).
#show occurs(object(robot,R), action(pickup, ()     ), T) :  pickup(robot(R),_,T).
#show occurs(object(robot,R), action(putdown,()     ), T) : putdown(robot(R),_,T).

#show occurs(object(robot,R), action(deliver,(O,A  )), T) : process(product(A),order(O),C,T), toShelf(S,C), position(robot(R),C,T-1).
#show occurs(object(robot,R), action(deliver,(O,A  )), T) : process(product(A),order(O),C,T), atShelf(S,C,T-1), carries(robot(R),S,T-1).
#show occurs(object(robot,R), action(deliver,(O,A  )), T) : process(product(A),order(O),C,T), position(S,C,T), carries(robot(R),S,T-1), shelved(S,product(A)).
#show occurs(object(robot,R), action(deliver,(O,A,N)), T) : process(product(A),order(O),S,N,T), carries(robot(R),S,T-1).