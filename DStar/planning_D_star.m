clc;
m = Map(20, 20);
m.set_obstacle([4 3; 4 4; 4 5; 4 6; 5 3; 6 3; 7 3]);
m.print_map();

start = m.map(2, 3);
goal = m.map(17, 11);
d = D_Star(m);
d.run(start, goal);
m.print_map();