#! /usr/bin/python

header_file = "./blast_config/include/command_list.h"
start = False
header_single_commands = []
for line in open(header_file, 'r'):
    if line.startswith("enum singleCommand"):
        start = True
    if start:
        if "}" in line:
            break
        line = line.replace("\n", '')
        line = line.replace(" ", '')
        line = line.replace("\t", '')
        words = line.split(",")
        header_single_commands.extend(words)

start = False
header_multi_commands = []
for line in open(header_file, 'r'):
    if line.startswith("enum multiCommand"):
        start = True
    if start:
        if "}" in line:
            break
        line = line.replace("\n", '')
        line = line.replace(" ", '')
        line = line.replace("\t", '')
        words = line.split(",")
        header_multi_commands.extend(words)

header_single_commands = [element for element in header_single_commands if not element.startswith("enum")]
header_single_commands = [element for element in header_single_commands if (element != "")]
header_multi_commands = [element for element in header_multi_commands if not element.startswith("enum")]
header_multi_commands = [element for element in header_multi_commands if (element != "")]

#print header_single_commands
#print header_multi_commands

#command_list_file = "./blast_config/command_list.c"
command_list_file = "./mcp/commanding/commands.c"

if False:
        s_commands_not_in_main_list = []
        start = False;
        for single_command in header_single_commands:
                command_found = False
                for line in open(command_list_file, 'r'):
                    if "MultiCommand" in line:
                        start = True
                    if start:
                        #if "COMMAND" in line:
                        if "case" in line:
                            if single_command in line:
                                command_found = True
                                print 'I found command', single_command, 'in line: ', line
                    if start and ("default:" in line):
                        start = False
                if not command_found:
                        s_commands_not_in_main_list.append(single_command)

#print 'single commands in MultiCommands in commands.c file:', s_commands_not_in_main_list

if True:
        m_commands_not_in_main_list = []
        start = False;
        for multi_command in header_multi_commands:
                command_found = False
                for line in open(command_list_file, 'r'):
                    #if "COMMAND" in line:
                    if "SingleCommand" in line:
                        start = True
                    if start:
                        if "case" in line:
                            if multi_command in line:
                                    command_found = True
                                    print 'I found command', multi_command, 'in line: ', line
                    if start and ("default:" in line):
                        start = False
                if not command_found:
                        m_commands_not_in_main_list.append(multi_command)

        #print '\nmulti commands in SingleCommand in command.c file:', m_commands_not_in_main_list
