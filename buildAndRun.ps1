# This script will automatically build the program again and start a server
# The server remains alive until the user hits enter.

$userinput = " "
while(-not ($userinput -contains "q")){
    echo "building program"
    cd ./src
    emcc simulator_web_interface.c -o simulator_web_interface.js -O2 -s EXPORTED_FUNCTIONS='["_simulate", "_set_pilot_input", "_retrieve_state_variable", "_retrieve_simulation_time", "_main"]' -s WASM=1 -s EXPORTED_RUNTIME_METHODS='["ccall", "cwrap"]' 
    cd ./..
    echo "program complete"

    echo "starting server"
    ./startServer.ps1

    $userinput = read-host -prompt "Hit enter to repeat loop, q+enter to quit"

    echo "shutting down server"

    stop-job "server"
}
echo "shutdown complete"