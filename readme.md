### This project was developed as a part of the Software Engineering program of  Ben-Gurion university 
***
## Mapping buildings with autonomous remote control cars - Description

The project main goal is to map buildings with the help of autonomous remote control cars, equipped with high resolution camera and ROS based computer.
This package deals with the AI module of the project, which upon receiving a 2D array, returns the best coordinate to travel to.

- Modular classes enable the implementations if various strategies of finding the most optimal coordinate.
- Flexible and adaptable implementation of the board.
 
## Motivation

This package is a part of software project directed by Ben Gurion University. The project combines both Software Engineering department and Electrical Engineering department.

Knowing the surface structure is crucial and critical routine of many professionals such as firefighters and security forces, receiving information about the operating   surface in the most efficient and accurate manner is often a matter of life or death.
 
The goal of our project is to map structures using vehicles and creating a computer model that describes, as accurate as possible ,the operating surface. When being activated the vehicle, which is equipped with various cameras and sensors, will be located at the entrance of the structure and will navigating autonomously in order to create the most accurate model of the structure. By doing so, this model can help saving human life when entering into dangerous areas.
***

## Installation

Deployment is very simple, just extract the compressed file into the designated  computer.
car's computer should have all of the files except for `client_app.py` under the same folder, where the `client_app.py` file should be located at the client-side computer.

## Running the program
 As mentioned before, this system is divided into car-side and a client side.
Running of the driver-side components is done with the execution of `init.sh` which takes care of running all of the built-in
classes.

all configuration that can be altered are inside the file named `conf` and can be edited in form of a Json (later will be translated to python dictionary using `ast` library.
The configuration file is being used by both car-side and client-side programs

In order to run the client-side program
```
python client_app.py
```
In order to stop the  whole process, run the script `stop.sh` which will close all of the driver-side terminals.


## Operating the scanning process

The whole scanning process is being controlled by the client-side terminal (will be the base for app development).
After a successful connection to the car-side, type `start` to start the scanning process after which direction will appear on the screen.
Press `ok` if following the direction is done successfully, press `fail` if the steps were not fulfilled completely.
for stopping the scanning process press `stop`


## Benchmarks and AI engines

Currently there are 3 main computing engines that compute the most optimal coordinate to visit by different factors.
###### engine V1: Will always choose the cell marked as 'unmapped' with the greatest distance from the car
###### engine V2: Will prefer the cell marked as unmapped that is located in front of the car's direction and has the greatest distance
###### engine V3: Very similar to engine V2 but will prefer the cell that also has the greatest number of unmapped neighbors



## Contributors

This whole project is written by the following members:
* Gal Porat
* Osher Damari
* Tomer Belzer

