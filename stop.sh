#!/bin/bash

for pid in `ps -ef | grep xterm | awk '{print $2}'` ;
 	do kill $pid ;
done
