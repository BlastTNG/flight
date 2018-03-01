#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <getopt.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <dirent.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h> // threads
#include <openssl/md5.h>

#include "netcmd.h"
#include "FIFO.h"
#include "cmdparser.h"
#include "telemparser.h"
#include "linkparser.h"

char * cmdfile = "/data/etc/bit_config/cmdlist.bit";
char * telemfile = "/data/etc/bit_config/telemlist.bit";
char * linklistdir = "/data/etc/bit_config/linklists/";

int main(int argc, char *argv[])
{
	int i, r;
  uint8_t format_serial[MD5_DIGEST_LENGTH] = {0};

	INCLUDE_ALL_FRAME = 1;

  // load command list from text file
  if (parse_cmdlist(cmdfile) < 0)
  {
    printf("Command parser error.\n");
    exit(3);
  }
  // load telemetry list from text file
  if (parse_telemlist(telemfile,"/data/etc/bit_config/formatfile.test") < 0)
  {
    printf("Telemetry parser error.\n");
    exit(3);
  }
  parse_formatfile("/data/etc/bit_config/formatfile.test");
  memcpy(format_serial,telem_serial,MD5_DIGEST_LENGTH);
  parse_telemlist(telemfile,NULL);

	if (load_all_linklists(linklistdir) < 0)
  {
    printf("Unable to load linklists\n");
    exit(3);
  }

/*
	no_auto_min_checksum = 1;
	all_frame_size = write_all_frame(NULL,NULL);	

	for (i=0;i<num_linkfile;i++)
	{
		char name[80];
		sprintf(name,"tmp-%d.Xll0",i);
		generate_linklist_formatfile(downlinklist[i],name);
		struct link_list * ll;
		if (i==0) ll = telemlist_to_linklist();
		else ll = parse_linklist(name);
		if (check_formatfile(name,*((uint16_t *) ll->serial)) == 2)
		{
			printf("Format file is verfied\n");
		}
		else
		{
			printf("Format file is inconsistent\n");
		}
		unlink(name);
	}
	no_auto_min_checksum = 0;
*/	

	int cmd=0;
	int tlm=0;
	int ll=0;

	for (i=1;i<argc;i++)
	{
		if (strcmp(argv[i],"-cmd") == 0) cmd = 1;
		else if (strcmp(argv[i],"-tlm") == 0) tlm = 1;
		else if (strcmp(argv[i],"-ll") == 0) ll = 1;
		else
		{
			printf("Invalid option %s (valid options are -cmd, -tlm, -ll)\n",argv[i]);
			exit(1);
		}
	}

	if (cmd)
	{
	printf("------------------------ CMDLIST START ------------------------\n");


  // print out command_list result
  printf("N_GROUPS=%d\nN_SCOMMANDS=%d\nN_MCOMMANDS=%d\n",N_GROUPS,N_SCOMMANDS,N_MCOMMANDS);
  
  for (i=0;i<N_GROUPS;i++) printf("Name=%s ID=0x%.8x\n",GroupNames[i],1 << i);
  for (i=0;i<N_SCOMMANDS;i++) 
  {
    if (scommands[i].command == i) printf("Command=%d ",scommands[i].command);
    else printf("Command=0x%.4x ",scommands[i].command/256);
    printf("Name=%s Groups=0x%.8x About=%s\n",scommands[i].name,scommands[i].group,scommands[i].about);
  }
  for (i=0;i<N_MCOMMANDS;i++)
  {
    if (mcommands[i].command == i) printf("Command=%d ",mcommands[i].command);
    else printf("Command=0x%.4x ",mcommands[i].command/256);
    printf("Name=%s Groups=0x%.8x NumParams=%d About=%s\n",mcommands[i].name,mcommands[i].group,mcommands[i].numparams,mcommands[i].about);
    for (r=0;r<mcommands[i].numparams;r++)
    {
      printf("\tName=%s Type=%c Min=%f Max=%f Field=%s\n",mcommands[i].params[r].name,mcommands[i].params[r].type,mcommands[i].params[r].min,mcommands[i].params[r].max,mcommands[i].params[r].field);
			if (mcommands[i].params[r].nt != NULL)
			{
				int j = 0;
				printf("\t\tList: ");
				while (mcommands[i].params[r].nt[j])
				{
					printf("%s ",mcommands[i].params[r].nt[j]);
					j++;
				}
				printf("\n");
			}
    }
  }

  printf("Command defaults (%d):\n",N_PARAMS);
  for (i=0;i<N_PARAMS;i++)
  {
    printf("\tCommand=%s Field=%s Type %c min=%f max=%f ",cmddefs[i].command, cmddefs[i].field, cmddefs[i].type, cmddefs[i].param->min, cmddefs[i].param->max);
    if (cmddefs[i].type == 'l') printf("Value=%d\n",cmddefs[i].value.l);
    else if (cmddefs[i].type == 'i') printf("Value=%d\n",cmddefs[i].value.i);
    else if (cmddefs[i].type == 'f') printf("Value=%f\n",cmddefs[i].value.f);
    else if (cmddefs[i].type == 'd') printf("Value=%f\n",cmddefs[i].value.d);
  }

  printf("Number of breaks %d:\n",N_BREAK);
  printf("\tSingle commands: ");
  for (i=0;i<N_BREAK;i++) printf("%d ", scom_break[i]);
  printf("\n\tMulti commands: ");
  for (i=0;i<N_BREAK;i++) printf("%d ", mcom_break[i]);
  printf("\n");
	for (i=0;i<N_BREAK;i++) printf("%d ", par_break[i]);
	printf("\n");
	}


  char str[80] = {0};
  for (i=0;i<MD5_DIGEST_LENGTH;i++) sprintf(str,"%s%.02x",str,command_list_serial[i]);
  
  printf("Command List Serial: %s\n",str);


	if (tlm)
	{
  printf("------------------------ TELEMLIST START ------------------------\n");
	   
  // print out telemetry result
  printf("N_TLM_TYPES=%d\n",N_TLM_TYPES);
  printf("format_id=%.4x\n",format_id);
  printf("frame_size=%d\n",frame_size);
  printf("num_frames=%d\n",num_frames);
  for (i=0;i<N_TLM_TYPES;i++) printf("Name=%s id=%d size=%d start=%d freq=%d skip=%d min=%f max=%f\n",bit_tlms[i].name,bit_tlms[i].id,bit_tlms[i].size,bit_tlms[i].start,bit_tlms[i].freq,bit_tlms[i].skip,bit_tlms[i].min,bit_tlms[i].max);
  printf("%d blocks of size:\n",N_BLOCK);
  for (i=0;i<N_BLOCK;i++) printf("%d, ",block_sizes[i]);
  printf(" = %d\n",frame_size);
	}

  char ser[80] = {0};
  for (i=0; i<MD5_DIGEST_LENGTH; i++) sprintf(ser,"%s%.02x", ser, telem_serial[i]);	

  printf("Telemetry list serial: %s\n",ser);
  
	ser[0] = 0;
	for (i=0; i<MD5_DIGEST_LENGTH; i++) sprintf(ser,"%s%.02x", ser, format_serial[i]);
	
	printf("Format file serial: %s\n",ser);



	if (ll)
	{
  printf("------------------------ LINKLIST START ------------------------\n");

	for (r=0;r<num_linkfile;r++)
	{
		printf("Press ENTER\r");
		getchar();
		int runningsum = 0;
	
		struct link_list * ll = downlinklist[r];  

		printf("\nLINKLIST \"%s\":\n",linkfile[r]);

	  // print result
	  for (i=0;i<ll->n_entries;i++)
 	 	{
    	if (ll->items[i].tlm != NULL)
    	{

      	printf("name = %s, start = %d, blk_size = %d, num = %d, comp_type = %d\n",
         ll->items[i].tlm->name, ll->items[i].start, ll->items[i].blk_size, ll->items[i].num,
         ll->items[i].comp_type);
				runningsum += ll->items[i].blk_size;
    	}
    	else 
			{
				printf("//--------CHECKSUM (over %d bytes)---------//\n",runningsum);
				runningsum = 0;
			}
  	}

		printf("Number of data blocks: %d\n",ll->num_blocks);

		for (i=0;i<ll->num_blocks;i++)
		{
			printf("%s == %d, alloc_size = %d\n",ll->blocks[i].name, ll->blocks[i].intname,ll->blocks[i].alloc_size);
		}

  	printf("Serial: ");
  	for (i=0;i<MD5_DIGEST_LENGTH;i++) printf("%x",ll->serial[i]);
  	printf("\n");
  	printf("n_entries = %d, blk_size = %d\n",ll->n_entries,ll->blk_size);
	}
	printf("all_frame_size = %d\n",all_frame_size);

	}



}
