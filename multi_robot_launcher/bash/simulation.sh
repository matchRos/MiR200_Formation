filename=$1
terminator -e "./simulation_enviroment.sh $filename" &
terminator -e "./system_handling.sh $filename"