#! /bin/bash

STOP_CMD="sudo pkill blast_inf_loop; sudo kill -INT \$(pidof mcp)"

ssh -t fc1user@fc1 "$STOP_CMD"
ssh -t fc1user@fc2 "$STOP_CMD"
