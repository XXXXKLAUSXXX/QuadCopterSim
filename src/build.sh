#!/bin/bash
emcc simulator_web_interface.c -o simulator_web_interface.js -O2 -s EXPORTED_FUNCTIONS='["_simulate", "_set_pilot_input", "_retrieve_state_variable", "_retrieve_simulation_time", "_main"]' -s WASM=1 -s EXPORTED_RUNTIME_METHODS='["ccall", "cwrap"]'

#run
python3 -m http.server 8084 &
firefox -private-window http://0.0.0.0:8084/index.html

