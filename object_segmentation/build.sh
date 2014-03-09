#!/usr/bin/bash
g++ `pkg-config --cflags --libs opencv` farneback.cpp -o farneback