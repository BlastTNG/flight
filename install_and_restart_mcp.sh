#! /bin/bash

TMP_DIR="/tmp"
INSTALL_DIR="/usr/local/bin"

INF_LOOP_CHECK="cd; sudo pkill blast_inf_loop; sudo nohup ./start_blast_inf_loop > /dev/null 2> /dev/null < /dev/null"
KILL_MCP="sudo kill -INT \$(pidof mcp) > /dev/null 2>&1"
INSTALL_MCP="install -m 755 -p $TMP_DIR/mcp $INSTALL_DIR/" 

scp mcp fc1user@fc1:$TMP_DIR/ 
scp mcp fc1user@fc2:$TMP_DIR/

ssh -t fc1user@fc1 "$INF_LOOP_CHECK; $INSTALL_MCP; $KILL_MCP"
ssh -t fc1user@fc2 "$INF_LOOP_CHECK; $INSTALL_MCP; $KILL_MCP"
