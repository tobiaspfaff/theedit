#!/bin/bash

cd build && cmake .. -DDEBUG=OFF && make -j4 && cd .. && bin/simedit $1 $2 $3 $4 $5 $6

