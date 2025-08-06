# KNOW-DEMO

ROS package to create, store, and utilize demonstration-based episodic memories as an ontological knowledge base. This package enables robots to record, organize, and reason about sequential demonstrations of tasks (episodic memories) using structured ontological frameworks, supporting applications like robot learning, behavior introspection, and adaptive long-term task execution.

This package allows to run the knowledge base using rosprolog, which includes features to be used within ROS (e.g., one can query the knowledge base calling a ROS service).


### Python3 virtual environment configuration and dependencies

In the package, we already provide a virtual environment but it was built for our computer and it will probably not work in yours. You can easily delete it and create and configure your own environment executing the following commands in a terminal (note that python3-venv needs to bee installed).

```
cd <know_demo_folder>/python_environment
python3 -m venv know_demo_venv
source know_demo_venv/bin/activate

python3 -m pip install rospkg pyyaml numpy pandas future
```


### Running a generic knowledge base

This example runs a simple knowledge base with general ontologies (e.g. dul, knowrob, soma, ocra, etc.) and including minimal semantic instances in it (e.g. a robot and a human). 

```
roslaunch know_demo generic_semantic_map.launch 
``` 

### Running an example to populate a generic knowledge base with episodic demonstrations

This example runs a node that reads some target knowledge about demonstrations and stores it in a generic knowledge base with general ontologies and including minimal semantic instances in it (e.g. a robot and a human). 

```
roslaunch know_demo memory_generation_with_generic_map.launch 
``` 

### Running an example to load a generic knowledge base with episodic demonstrations

This example runs a node that reads an episodic memory containing knowledge about demonstrations and loads it in a generic knowledge base with general ontologies and including minimal semantic instances in it (e.g. a robot and a human). 

```
roslaunch know_demo memory_loading_with_generic_map.launch 
``` 
