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

#include "calibrate.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "blast.h"

int main(int argc, char *argv[])
{
  if (argc == 1) {
    printf("Need to specify linklist directory\n");
    return 1;
  }
  char * linklistdir = argv[1];

	int i, r;
  uint8_t format_serial[MD5_DIGEST_LENGTH] = {0};
  linklist_t *ll_array[MAX_NUM_LINKLIST_FILES] = {NULL};

  channels_initialize(channel_list);
  linklist_assign_channel_list(channel_list);

	if (load_all_linklists(linklistdir, ll_array) < 0)
  {
    printf("Unable to load linklists\n");
    exit(3);
  }

  printf("------------------------ LINKLIST START ------------------------\n");

	linklist_t * ll = ll_array[0];  
  r = 0;

	while (ll)
	{
		int runningsum = 0;

		printf("\n================LINKLIST \"%s\"==================\n",ll->name);

	  // print result
	  for (i=0;i<ll->n_entries;i++)
 	 	{
    	if (ll->items[i].tlm != NULL)
    	{

      	printf("name = %s, start = %d, blk_size = %d, num = %d, comp_type = %d\n",
         (ll->items[i].tlm->field[0]) ? ll->items[i].tlm->field : "BLOCK", ll->items[i].start, 
         ll->items[i].blk_size, ll->items[i].num, ll->items[i].comp_type);
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
			printf("%s, id=%d, alloc_size = %d\n",ll->blocks[i].name, ll->blocks[i].id,ll->blocks[i].alloc_size);
		}

  	printf("Serial: ");
  	for (i=0;i<MD5_DIGEST_LENGTH;i++) printf("%x",ll->serial[i]);
  	printf("\n");
  	printf("n_entries = %d, blk_size = %d\n",ll->n_entries,ll->blk_size);
    ll = ll_array[++r];

	}


  printf("\n===================GENERAL INFO====================\n");

	printf("all_frame_size = %d\n",allframe_size);

}
