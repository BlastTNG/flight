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

#include "blast.h"
#include "calibrate.h"
#include "channels_tng.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "linklist_writer.h"

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
  superframe_entry_t * superframe_list = NULL;

  channels_initialize(channel_list);
  define_allframe(superframe);

  printf("Superframe size = %d, count = %d, serial = %.8lx\n", superframe->size, superframe->n_entries, superframe->serial);

  write_superframe_format(superframe, "superframe.txt");
  superframe_t * testsf = parse_superframe_format("superframe.txt"); 
  printf("Parse serial = %.8lx\n", testsf->serial);

	if (load_all_linklists(superframe, linklistdir, ll_array) < 0)
  {
    printf("Unable to load linklists\n");
    exit(3);
  }

  // check the parser and writer
	linklist_t * ll = ll_array[0];  
  r = 0;
  while (ll) {
    printf("Checking %s...\n", ll->name);
    write_linklist_format(ll, ll->name);
    linklist_t * temp_ll = parse_linklist_format_opt(superframe, ll->name, LL_NO_AUTO_CHECKSUM);
    printf("0x%.4x == 0x%.4x\n", *(uint32_t *) ll->serial, *(uint32_t *) temp_ll->serial);

    delete_linklist(temp_ll);
    ll = ll_array[++r];
  } 


  printf("------------------------ LINKLIST START ------------------------\n");

	ll = ll_array[0];  
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

      	printf("name = %s, start = %d, blk_size = %d, num = %d, comp_type = %s, sf_start = %d, sf_skip = %d\n",
         (ll->items[i].tlm->field[0]) ? ll->items[i].tlm->field : "BLOCK", ll->items[i].start, 
         ll->items[i].blk_size, ll->items[i].num, (ll->items[i].comp_type == NO_COMP) ? "NONE" : compRoutine[ll->items[i].comp_type].name, 
         ll->items[i].tlm->start, ll->items[i].tlm->skip);
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

	printf("allframe_size = %d\n",superframe->allframe_size);

}
