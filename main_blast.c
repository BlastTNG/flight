#include "main_blast.h"

superframe_t groundhog_init_superframe() {
  channels_initialize(channel_list);
}

linklist_t ** groundhog_init_linklists(char * filedir, int flags) {
  linklist_t ** ll_list = calloc(MAX_NUM_LINKLIST_FILES, sizeof(linklist_t *));
  load_all_linklists(superframe, filedir, ll_list, flags);
  generate_housekeeping_linklist(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), ALL_TELEMETRY_NAME);
  linklist_generate_lookup(ll_list);  
  write_linklist_format(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), DEFAULT_LINKLIST_DIR ALL_TELEMETRY_NAME ".auto");

  channels_write_calspecs("test.cs", derived_list);

  return ll_list;
}
