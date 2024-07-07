This project is about, path palnning in an environment with dynamically moving obstacles, from start to goal. 

![image](https://github.com/saidineshgelam/RT-RRT---A-REAL-TIME-PATH-PLANNING-ALGORITHM/assets/144295692/61dd8159-0a33-4dd5-b79c-b52fe79b1d6f)


This is done in two methods namely, "method_1.py" and "method_2.py" (suggested in the paper )" 

Method_1: Generalized path palnning withot localized planning. Just run this code you can see the agent moving from start to goal finding entirely a new path every time it meets the obstacle. The time taken and nodes explored are prompted as outputs.



Method_2 : This is a optimized method mentioned in the paper, which finds a new subpart to escape the obstacle and joins the main path again  instead of building the entire new path again. Just run this code you can see agent moving from start to goal finding sub-paths instead of finding a new tree again.

You can compare efficiency of both the methodes in terms of number of nodes explored
