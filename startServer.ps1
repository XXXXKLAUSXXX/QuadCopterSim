# This script starts a server in the background of the current shell. 
# The server terminates when the current process does. I recommend using 'read-host' to wait for the user to hit enter to prevent a premature shutdown.

cd ./src

start-job -scriptblock {
    set-location $input
    python3 -m http.server 8084
} -name "server" -InputObject $pwd
cd ./.. 