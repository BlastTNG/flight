#ifndef MCP_H
#define MCP_H

#define MCP_INFO    0
#define MCP_STARTUP 1
#define MCP_SCHED   2
#define MCP_WARNING 3
#define MCP_ERROR   4
#define MCP_TFATAL  5
#define MCP_FATAL   6

void mprintf(int flag, char* fmt, ...);
void merror(int flag, char* fmt, ...);
void mputs(int flag, const char* message);

#endif
