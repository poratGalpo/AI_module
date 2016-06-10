### NOTE: This library is still under construction and testing
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

Deployment is very simple, just extract the compressed file into the destinated computer.
car's computer should have all of the files except for `client_app.py` under the same folder, where the `client_app.py` file should be located at the client-side computer.

## Running the program

As mentioned before, this system is devided into car-side and a client side.
all configuration that can be altered are inside the file named `conf` and can be edited in form of a Json (later will be translated to python dictionary using `ast` library.
The configuration file is being used by both car-side and client-side programs

in order to run the car-side program:
```
python driver.py
```

In order to run the client-side program
```
python client_app.py
```

## Tests

######   Still need to be written



## Benchmarks

Currently there are 3 main computing engines that compute the most optimal coordinate to visit by different factors.
After a full intgartion with the car, this part will be filled with efficiency of each of the computing engines.
######   Will be written after integration

## Contributors

This whole project is written by the following members:
* Gal Porat
* Osher Damari
* Tomer Belzer

