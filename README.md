This project is about, path palnning in an environment with dynamically moving obstacles, from start to goal. 



https://github.com/saidineshgelam/RT-RRT---A-REAL-TIME-PATH-PLANNING-ALGORITHM/assets/144295692/6da458f5-7a5f-4319-9089-938b676a3c71




This is done in two methods namely, "method_1.py" and "method_2.py" (suggested in the paper )" 

Method_1: Generalized path palnning withot localized planning. Just run this code you can see the agent moving from start to goal finding entirely a new path every time it meets the obstacle. The time taken and nodes explored are prompted as outputs.



https://github.com/saidineshgelam/RT-RRT---A-REAL-TIME-PATH-PLANNING-ALGORITHM/assets/144295692/1f66ba40-9acb-4a51-b309-34112cd20877



Method_2 : This is a optimized method mentioned in the paper, which finds a new subpart to escape the obstacle and joins the main path again  instead of building the entire new path again. Just run this code you can see agent moving from start to goal finding sub-paths instead of finding a new tree again.


https://github.com/saidineshgelam/RT-RRT---A-REAL-TIME-PATH-PLANNING-ALGORITHM/assets/144295692/43bcd280-0b07-4026-87ad-49558defb772


You can compare efficiency of both the methodes in terms of number of nodes explored
