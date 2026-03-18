cd ./src

start-job -scriptblock {
    set-location $input
    python3 -m http.server 8084
} -name "server" -InputObject "E:\repos\QuadCopterSim\src"
cd ./.. 