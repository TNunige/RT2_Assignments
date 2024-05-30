# Research Track 2 Assignment
## Assignment 1
### Properly Comment 
For the assignment 1, I created the software documentation of the second assignment of RESEARCH TRACK 1.
Since I used Python for the assignment, I used Sphinx for the documentation.

The documentation is here: https://tnunige.github.io/RT2_Assignments/

## Assignment 2
### Jupyter Notebook for interacting with the simulation
For the assignment 2, I created a Jupyter notebook to interact with the simulation of the second assignment of RESEARCH TRACK1. The notebook allows the user to assign or cancel the goals to the robot. And It visualizes the robot positions and goal positions, and shows the number of reached/unreached goals.

To use the Jupyter notebook:

1.Install Jupyter notebook with this command
```
pip3 install jupyter bqplot pyyaml ipywidgets
```
```
jupyter nbextension enable --py widgetsnbextension
```
2. run jupyter notebook server
```
jupyter notebook
```
3. Open the notebook file `User_Interface.ipynb`
   
4. Execute all the cells in the notebook to interact the simulation environment.
 
   Be sure to start the simulation before executing the notebook.
   
   To start the simulation, you have to go inside the ROS workspace directory, then run the ROS master with this:
   ```
   roscore
   ```
   Then, launch the simultion with the ros launch command:
   ```
   roslaunch assignment_2_2023 assignment1.launch
   ```
## Assignment 3
### Statistical Analysis of the first assignment of RESEARCH TRACK 1
For the assignment 3, I performed the statistical analysis of the first assignment of RESEARCH TRACK 1.
In the analysis, I compared the performance of the algorithm I implemented with the one implemented by one of my collegues. The purpose of the analysis is to decide which algorithm is better when tokens are randomly placed in the environment.

You can find the report of the analysis named `RT2_3rd_assignment.pdf`
