filename=$1
terminator -e "./simulation_term1.sh $filename" &
terminator -e "./simulation_term2.sh $filename"