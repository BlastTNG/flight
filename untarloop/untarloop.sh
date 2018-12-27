#!/bin/bash
inotifywait -m ./ -e create -e moved_to |
    while read path action file; do
        if [ ${file:0:10} = "file_block" ]; then 
            inotifywait -e close_write $file
            echo "Untarring..'$file'"
            tar -xf $file
            echo "Done"
            rm $file
        elif [ ${file: -6} = "tar.gz" ]; then 
            #inotifywait -e close $file
            echo "Untarring..'$file'"
            tar -xf $file
            echo "Done"
            rm $file
        fi
    done
