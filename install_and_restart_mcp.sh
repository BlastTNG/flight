#! /bin/bash

TMP_DIR="/tmp"
INSTALL_DIR="/usr/local/bin"

INF_LOOP_CHECK="if pidof blast_inf_loop > /dev/null 2>&1; then echo 'Inf loop already running'; else echo 'Starting inf loop'; sudo bash -c '/usr/local/bin/blast_inf_loop >/dev/null 2>&1 &' fi"
KILL_MCP="sudo kill -INT \$(pidof mcp)"
INSTALL_MCP="install -m 755 -p $TMP_DIR/mcp $INSTALL_DIR/" 

scp mcp fc1user@fc1:$TMP_DIR/ 
scp mcp fc1user@fc2:$TMP_DIR/

ssh -t fc1user@fc1 "$INSTALL_MCP; $KILL_MCP"
ssh -t fc1user@fc2 "$INSTALL_MCP; $KILL_MCP"
