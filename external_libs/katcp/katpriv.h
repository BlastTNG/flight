#ifndef _KATPRIV_H_
#define _KATPRIV_H_

#include <signal.h>
#include <stdarg.h>

#include <sys/time.h>
#include <sys/types.h>

#include <avltree.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KATCP_NAME_LENGTH     64

#define KATCL_IO_SIZE       4096  /* block we want to write out */
#define KATCL_BUFFER_INC     512  /* amount by which we resize read */
#define KATCL_ARGS_INC         8  /* grow the vector by this amount */

#define KATCL_PARSE_FRESH      0  /* newly allocated or cleared */
#define KATCL_PARSE_COMMAND    1  /* parsing first argument */
#define KATCL_PARSE_WHITESPACE 2  /* parsing between arguments */
#define KATCL_PARSE_ARG        3  /* parsing argument */
#define KATCL_PARSE_TAG        4  /* parsing optional tag */
#define KATCL_PARSE_ESCAPE     5  /* parsing escape sequence */
#define KATCL_PARSE_FAKE       6  /* generated manually, not parsed */
#define KATCL_PARSE_DONE       7  /* a complete message */


#define KATCL_ALIGN_NONE       0x0
#define KATCL_ALIGN_BYTE       0x1
#define KATCL_ALIGN_WORD       0x2  /* 32 bits */

/***************************************************************************/

struct katcl_byte_bit;

struct katcl_larg{
  unsigned int a_begin;
  unsigned int a_end;
  unsigned int a_escape;
};

struct katcl_queue
{
  struct katcl_parse **q_queue; /* parse array */
  unsigned int q_size;          /* size of queue */
  unsigned int q_head;          /* current position */
  unsigned int q_count;         /* No of entries */
};

struct katcl_parse{
  unsigned int p_magic;
  unsigned int p_state;

  char *p_buffer;
  unsigned int p_size;
  unsigned int p_have;
  unsigned int p_used;
  unsigned int p_kept;

  struct katcl_larg *p_args;
  struct katcl_larg *p_current;
  unsigned int p_count;
  unsigned int p_got;

  int p_refs;
  int p_tag;
};

struct katcl_line{
  int l_fd;

  struct katcl_parse *l_ready;
  struct katcl_parse *l_next;

  struct katcl_parse *l_stage;

  char l_buffer[KATCL_IO_SIZE];
  unsigned int l_pending;
  unsigned int l_arg;  /* argument */
  unsigned int l_offset; /* offset into argument */

  struct katcl_queue *l_queue;

  int l_error;
  int l_sendable;
};

/******************************************************************************/

struct katcp_dispatch;

struct katcp_cmd{
  char *c_name;
  char *c_help;
  int (*c_call)(struct katcp_dispatch *d, int argc);
  struct katcp_cmd *c_next;
  unsigned int c_mode;
  unsigned int c_flags;
};

/**********************************************************************/

#define KATCP_SENSOR_INVALID (-1)
#define KATCP_SENSOR_INTEGER  0 
#define KATCP_SENSOR_BOOLEAN  1
#define KATCP_SENSOR_DISCRETE 2
#define KATCP_SENSOR_LRU      3

#ifdef KATCP_USE_FLOATS
#define KATCP_SENSOR_FLOAT    4
#define KATCP_SENSORS_COUNT   5
#else
#define KATCP_SENSORS_COUNT   4
#endif

struct katcp_sensor;
struct katcp_nonsense;
struct katcp_acquire;

#ifdef KATCP_USE_FLOATS
struct katcp_double_acquire{
  double da_current;
  double (*da_get)(struct katcp_dispatch *d, struct katcp_acquire *a);
};
#endif

struct katcp_discrete_acquire{
  unsigned int da_current;
  int (*da_get)(struct katcp_dispatch *d, struct katcp_acquire *a);
};

struct katcp_integer_acquire{
  int ia_current;
  int (*ia_get)(struct katcp_dispatch *d, struct katcp_acquire *a);
};

struct katcp_acquire{
  struct katcp_sensor **a_sensors;
  unsigned int a_count;

  int a_type;
  int a_users;
  int a_periodics;          /* tracks if timer callback is running */

  struct timeval a_poll;    /* rate at which we poll this sensor */
  struct timeval a_current; /* current rate */
  struct timeval a_limit;   /* fastest update rate */
  struct timeval a_last;    /* last time value was acquired */
#ifdef KATCP_EXPERIMENTAL
  struct timeval a_real;    /* time which sensor was actually read */
#endif

  void *a_local;
  void (*a_release)(struct katcp_dispatch *d, struct katcp_acquire *a);

  void *a_more; /* could be a union */
};

struct katcp_sensor{
  int s_magic;
  int s_type;
  char *s_name;
  char *s_description;
  char *s_units;

  int s_preferred;

  int s_status; /* WARNING, etc */
  int s_mode; /* in what mode available */
  struct timeval s_recent;
#if 0
  int s_running; /* have it do stuff */
#endif

  unsigned int s_refs;
  struct katcp_nonsense **s_nonsense;

  struct katcp_acquire *s_acquire;

  int (*s_extract)(struct katcp_dispatch *d, struct katcp_sensor *sn);
  int (*s_flush)(struct katcp_dispatch *d, struct katcp_sensor *sn);
  void *s_more;
};

#ifdef KATCP_USE_FLOATS
struct katcp_double_sensor{
  double ds_current;

  int ds_checks;

  double ds_nominal_min;
  double ds_nominal_max;

  double ds_warning_min;
  double ds_warning_max;
};
#endif

struct katcp_integer_sensor{
  int is_current;

  int is_checks;

  int is_nominal_min;
  int is_nominal_max;

  int is_warning_min;
  int is_warning_max;
};

struct katcp_discrete_sensor{
  int ds_current;
  int ds_size;
  char **ds_vector;
};

struct katcp_nonsense{
  int n_magic;
  struct katcp_dispatch *n_client;
  struct katcp_sensor *n_sensor;
  int n_strategy;
  int n_status;
  struct timeval n_period;
  struct timeval n_next;
  int n_manual;

  void *n_more;
};

#ifdef KATCP_USE_FLOATS
struct katcp_double_nonsense{
  double dn_previous;
  double dn_delta;
};
#endif

struct katcp_integer_nonsense{
  int in_previous;
  int in_delta;
};

struct katcp_discrete_nonsense{
  unsigned int dn_previous;
};

#if 0
struct katcp_sensor_integer{
  int si_min;
  int si_max;
  int si_current;
  int (*si_get)(struct katcp_sensor *s, void *local);
};

struct katcp_sensor_discrete{
  int sd_current;
  char **sd_vector;
  unsigned int sd_size;
  int (*sd_get)(struct katcp_sensor *s, void *local);
};

struct katcp_nonsense_discrete{
  int nd_previous;
};

#ifdef KATCP_USE_FLOATS
struct katcp_sensor_float{
  double sf_min;
  double sf_max;
  double sf_current;
  double (*sf_get)(struct katcp_sensor *s, void *local);
};

struct katcp_nonsense_float{
  double nf_previous;
  double nf_delta;
};
#endif
#endif

/**********************************************************************/

struct katcp_entry{
  char *e_name;
  struct katcp_notice *(*e_prep)(struct katcp_dispatch *d, char *flags, unsigned int from, unsigned int to);

  int (*e_enter)(struct katcp_dispatch *d, struct katcp_notice *n, char *flags, unsigned int to);
  void (*e_leave)(struct katcp_dispatch *d, unsigned int from);

  void *e_state;
  void (*e_clear)(struct katcp_dispatch *d, unsigned int mode);
  unsigned int e_status;
#if 0
  char *e_version;
  unsigned int e_minor;
  unsigned int e_major;
#endif
};

struct katcp_version{
  char *v_label;
  char *v_value;
  char *v_build;
  unsigned int v_mode;
};

#if 0
#define KATCP_PS_UP    1
#define KATCP_PS_TERM  2
struct katcp_process{
  void (*p_call)(struct katcp_dispatch *d, int status);
  pid_t p_pid;
  char *p_name;
  int p_type;
  int p_state;
};
#endif

struct katcp_notice;

struct katcp_trap{
  char *t_name;
  struct katcp_notice *t_notice;
};

struct katcp_map{
  struct katcp_trap **m_traps;
  unsigned int m_size;
};

#ifdef KATCP_SUBPROCESS
struct katcp_job{
  unsigned int j_magic;
  struct katcp_url *j_url;

  pid_t j_pid;

  int j_state; /* state machine */
  int j_code; /* exit code */

  int j_recvr;  /* number of requests received */
  int j_sendr; /* number of requests sent */

  struct katcl_line *j_line;

  struct katcp_notice *j_halt;

  struct katcp_notice **j_queue; 
  unsigned int j_size;
  unsigned int j_head; /* points at the current head */
  unsigned int j_count; /* number of entries present */

  struct katcp_map *j_map;
};
#endif

#if 0
#define KATCP_TIME_OTHER   0
#define KATCP_TIME_CURRENT 1
#define KATCP_TIME_REFRESH 2
#define KATCP_TIME_REAP    3
#endif

struct katcp_time{
  int t_magic;

  struct timeval t_when;
  struct timeval t_interval;

  int t_armed;

  void *t_data;
  int (*t_call)(struct katcp_dispatch *d, void *data);
};

struct katcp_invoke{
  struct katcp_dispatch *v_client;
  void *v_data;
  int (*v_call)(struct katcp_dispatch *d, struct katcp_notice *n, void *data);
  int v_trigger;
};

struct katcp_notice{
  struct katcp_invoke *n_vector;
  unsigned int n_count;

#if 0
  int n_code;
#endif
  char *n_name;

  int n_trigger;
  int n_tag;
  int n_use;

#if 0
  /* might get away with only one parse structure */
  struct katcl_parse *n_parse;
#endif

  struct katcl_queue *n_queue;
  struct katcl_parse *n_parse;

#if 0
  int n_position;
#endif
  int n_changes;

#if 0
  void *n_target;
  int (*n_release)(struct katcp_dispatch *d, struct katcp_notice *n, void *target);
#endif
};

struct katcp_arb{
  char *a_name;
  int a_fd;
  
  unsigned int a_type;

  unsigned short a_reap;
  unsigned short a_mode;

  int (*a_run)(struct katcp_dispatch *d, struct katcp_arb *a, unsigned int mode);
  void *a_data;
};

#ifdef KATCP_EXPERIMENTAL

/* duplex structures: was supposed to be called duplex, but flat is punnier */
/********************************************************************/

#define KATCP_VRT_GONE    ((unsigned short)(-1))

#define KATCP_VRT_STRING    0
#define KATCP_VRT_TREE      1
#define KATCP_VRT_ARRAY     2
#define KATCP_MAX_VRT       3


/* flags need to be contigous, flag_from_string_vrbl_katcp needs that */

#define KATCP_VRF_NONE     0
#define KATCP_VRF_ENV   0x01  /* exported to environment */
#define KATCP_VRF_VER   0x02  /* a version variable */
#define KATCP_VRF_SEN   0x04  /* a variable visible as a sensor */
#define KATCP_VRF_FLX   0x08  /* type can change */
#define KATCP_VRF_HID   0x10  /* hidden */

#define KATCP_MASK_VRF  0x1f  /* mask of all flags */

/* other possible options */
#if 0
#define KATCP_VRF_COW         /* copy on write */
#define KATCP_VRF_RO          /* readonly */
#endif

#define KATCP_VRC_VERSION_VERSION ":version"
#define KATCP_VRC_VERSION_BUILD   ":build"

#define KATCP_VRC_SENSOR_VALUE    ":value"
#define KATCP_VRC_SENSOR_STATUS   ":status"
#define KATCP_VRC_SENSOR_HELP     ":help"
#define KATCP_VRC_SENSOR_UNITS    ":units"
#define KATCP_VRC_SENSOR_TYPE     ":type"
#define KATCP_VRC_SENSOR_RANGE    ":range"
#define KATCP_VRC_SENSOR_TIME     ":time"
#if 0 /* unused - causes unexpected namespace collisions */
#define KATCP_VRC_SENSOR_PREFIX   ":prefix"
#endif

struct katcp_vrbl_payload;

struct katcp_vrbl_map_item{
  char *m_key;
  struct katcp_vrbl_payload *m_value;
};

struct katcp_vrbl_array{
  struct katcp_vrbl_payload **a_elements;
  unsigned int a_size;
};

struct katcp_vrbl_payload{
  unsigned short p_type;
  union{
    char *u_string;
    unsigned int u_integer;
    struct avl_tree *u_tree;
    struct katcp_vrbl_array u_array;
  } p_union;
};

struct katcp_vrbl{
  char *v_name; /* not always set, never owned */

  unsigned short v_status;
  unsigned short v_flags;

#if 1
  /* unsure where this is going to be used */
  int (*v_refresh)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx);
#endif
  int (*v_change)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx);
  void (*v_release)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx);

  struct katcp_vrbl_payload *v_payload;

  void *v_extra;
};

struct katcp_region{
  struct avl_tree *r_tree;
};

struct katcp_subscribe{
  struct katcp_vrbl *s_variable;
  struct katcp_endpoint *s_endpoint;
  unsigned int s_strategy;

  /* for time, last time updated, next time when ... */
};

struct katcp_wit{
  unsigned int w_magic;
  struct katcp_endpoint *w_endpoint;
  struct katcp_subscribe **w_vector;
  unsigned int w_size;

  /* todo - timeval counter ... runs at rate */
};

struct katcp_listener{
  unsigned int l_magic;
  unsigned int l_port;
  char *l_address;

  struct katcp_group *l_group;
};

struct katcp_cmd_item{
  /* a single command */
  char *i_name;
  char *i_help;
  int (*i_call)(struct katcp_dispatch *d, int argc);
  unsigned int i_flags; 
  void *i_data;
  void (*i_clear)(void *data);
  int i_refs;
};

struct katcp_cmd_map{
  /* "table" of commands */
  char *m_name;
  unsigned int m_refs;
  struct avl_tree *m_tree;
  struct katcp_cmd_item *m_fallback;
};

#define KATCP_SCOPE_INVALID    (-1)
/* dpx-misc has a lookuptable based on this order */
#define KATCP_SCOPE_SINGLE       0
#define KATCP_SCOPE_GROUP        1
#define KATCP_SCOPE_GLOBAL       2
#define KATCP_MAX_SCOPE          3

#define KATCP_MAP_UNSET        (-1)

#define KATCP_MAP_INNER_REQUEST  0
#define KATCP_MAP_REMOTE_REQUEST 1
#define KATCP_MAP_INNER_INFORM   2
#define KATCP_MAP_REMOTE_INFORM  3

#define KATCP_SIZE_MAP  4

#define KATCP_DIRECTION_INVALID (-1)

#define KATCP_DIRECTION_INNER     0
#define KATCP_DIRECTION_REMOTE    1

/* discard entire pending set after this many queued elements */
#define KATCP_FLUSH_DEFER         8

struct katcp_group{
  /* a set of flats which belong together, probably spawned off the same listener, probably same set of commands, probably same "mode" */
  char *g_name;
  unsigned int g_flags;
  struct katcp_cmd_map *g_maps[KATCP_SIZE_MAP];

  struct katcp_flat **g_flats;
  unsigned int g_count;

  int g_log_level;
  int g_scope;

  int g_use;             /* are we ref'ed by the listener */
  int g_autoremove;      /* do we disappear if use and count are zero ? */

  int g_flushdefer;      /* number of queued items after we cancel the lot */

  struct katcp_region *g_region;
};

#define KATCP_REPLY_HANDLE_REPLIES   0x1
#define KATCP_REPLY_HANDLE_INFORMS   0x2

struct katcp_response_handler{
  unsigned int r_flags;
  char *r_message;
  int (*r_reply)(struct katcp_dispatch *d, int argc);
  struct katcp_endpoint *r_issuer;
  struct katcp_endpoint *r_recipient;
  struct katcl_parse *r_initial;
};

#define KATCP_SIZE_REPLY         2


#define KATCP_DPX_SEND_INVALID   0x00

#define KATCP_DPX_SEND_DESTINATION 0x0f
#define KATCP_DPX_SEND_NULL        0x01  /* throw IO away */
#define KATCP_DPX_SEND_STREAM      0x02  /* goes to f_line */
#define KATCP_DPX_SEND_PEER        0x03  /* goes to top of endpoint */

#define KATCP_DPX_SEND_PERSISTENCE 0xf0
#define KATCP_DPX_SEND_CURRENTLY   0x10  /* set until callback finishes */
#define KATCP_DPX_SEND_LOCKED      0x20  /* user set indefinitely */

#define KATCP_DPX_SEND_RESET      0x100  /* clear lock flag */

#define KATCP_DEFER_OUTSIDE_REQUEST  0x1  /* stop handling outside requests, already have one */
#define KATCP_DEFER_OWN_REQUEST      0x2  /* don't send further requests, other side already busy */

#define KATCP_STALE_SENSOR_NAIVE     0x1  /* haven't issued a sensor-list, can't be out of date */
#define KATCP_STALE_SENSOR_CURRENT   0x2  /* have done sensor-list recently */
#define KATCP_STALE_SENSOR_STALE     0x3  /* things have changed since last sensor-list */
#define KATCP_STALE_MASK_SENSOR      0x3  /* mask out sensor related data */

struct katcp_flat{
  /* a client instance, intended to replace what was job and dispatch previously */
  unsigned int f_magic;
  char *f_name;          /* locate the thing by name */

  unsigned int f_flags;           /* which directions can we do */

  int f_state;           /* up, shutting down, etc */
  int f_exit_code;       /* reported exit status */

  int f_log_level;       /* log level currently set */

  int f_scope;           /* how much we see */

  unsigned int f_stale;          /* what is out of date */

  unsigned int f_deferring;      /* status flags for requests outstanding */
  struct katcl_gueue *f_defer;   /* deferred requests */
  int f_max_defer;               /* worst case queue size */

  struct katcp_endpoint *f_peer;   /* queue for all messages, can be from different senders */
  struct katcp_endpoint *f_remote; /* queue for remote messages, used to make remote messages "fit" into peer queue */

  struct katcl_line *f_line;
  struct katcp_shared *f_shared;

  struct katcl_parse *f_orx;     /* originating received message (needed for replies) */
  struct katcl_parse *f_rx;      /* received message */
  struct katcl_parse *f_tx;      /* message about to send */

  struct katcp_cmd_item *f_cmd;  /* command currently matched */

#if 0
  unsigned int f_send;           /* message sending flags */
#endif

  struct katcp_response_handler f_replies[KATCP_SIZE_REPLY]; /* this should probably be a dynamic number */
 
  struct katcp_endpoint *f_current_endpoint;

  struct katcp_cmd_map *f_maps[KATCP_SIZE_MAP];
  int f_current_map; 

  struct katcp_group *f_group;

  /* TODO: */
  
  /* notices, sensors */

  /* a sensor could probably be a special type of notice */

  struct katcp_region *f_region;
};
#endif

#define KATCP_FLAT_STACK 4

struct katcp_shared{
  unsigned int s_magic;
  struct katcp_entry *s_vector;
  unsigned int s_default; /* default log level */
  unsigned int s_size;
#if 0
  unsigned int s_modal;
#endif

  int (*s_prehook)(struct katcp_dispatch *d, int argc);
  int (*s_posthook)(struct katcp_dispatch *d, int argc);

  struct katcp_cmd *s_commands;
  struct katcp_sensor *s_mode_sensor;
  unsigned int s_mode;
  unsigned int s_flaky; /* mode transition failed, breaking the old one */

  unsigned int s_new;
  char *s_options;
  struct katcp_notice *s_transition;

  struct katcp_dispatch *s_template;
  struct katcp_dispatch **s_clients;

  unsigned int s_count;
  unsigned int s_used;

  int s_lfd;

  struct katcp_job **s_tasks;
  unsigned int s_number;

#if 0
  struct katcp_process *s_table;
  int s_entries;
#endif

  struct katcp_time **s_queue;
  unsigned int s_length;

  struct katcp_arb **s_extras;
  unsigned int s_total;

  struct katcp_notice **s_notices;
  unsigned int s_pending;

  unsigned int s_busy; /* more things to do, keep select short */

  struct katcp_group **s_groups;
  struct katcp_group *s_fallback;
  unsigned int s_members;
  unsigned int s_lock; /* can't call functions which mess with group pointers */

  struct katcp_flat **s_this;
  int s_level;
  int s_stories;

  unsigned int s_changes;

  struct katcp_endpoint *s_endpoints;

  struct katcp_region *s_region;

#if 0
  int s_version_major;
  int s_version_minor;
  char *s_version_subsystem; /* not ideally named */
#endif

  char **s_build_state;
  int s_build_items;

  struct katcp_sensor **s_sensors;
  unsigned int s_tally;

  struct katcp_version **s_versions;
  unsigned int s_amount;

  sigset_t s_signal_mask;
  struct sigaction s_child_current, s_child_previous;
  struct sigaction s_term_current, s_term_previous;
  int s_restore_signals;

  fd_set s_read, s_write;
  int s_max;
  
  struct katcp_type **s_type;
  unsigned int s_type_count;

  time_t s_start;
};

struct katcp_dispatch{
  int d_level; /* log level */
  int d_ready;
  int d_run; /* 1 if up, -1 if shutting down, 0 if shut down */
  int d_exit; /* exit code, reason for shutting down */
  int d_pause; /* waiting for a notice */
  struct katcl_line *d_line;

  int (*d_current)(struct katcp_dispatch *d, int argc);

  struct katcp_shared *d_shared;

  struct katcp_nonsense **d_nonsense;
  unsigned int d_size;

  struct katcp_notice **d_notices;
  unsigned int d_count;

  struct katcp_notice *d_end;

  int d_clone;

  char d_name[KATCP_NAME_LENGTH];
};

struct katcp_url {
  int u_use;
  char *u_str;
  char *u_scheme;
  char *u_host;
  int u_port;
  char **u_path;
  int u_pcount;
  char *u_cmd;
};

struct katcp_type {
  char *t_name;
  
  int t_dep;

  struct avl_tree *t_tree;

  void (*t_print)(struct katcp_dispatch *, char *key, void *);
  void (*t_free)(void *);
  int  (*t_copy)(void *src, void *dest, int);
  int  (*t_compare)(const void *, const void *);
  void *(*t_parse)(struct katcp_dispatch *d, char **);
  char *(*t_getkey)(void *data);
};

struct katcp_tobject {
  void *o_data;
  struct katcp_type *o_type;
  int o_man;
};

struct katcp_stack {
  struct katcp_tobject **s_objs;
  int s_count;
};

#ifdef KATCP_SUBPROCESS
struct katcp_dynamic_mode{
  int d_magic;
  char *d_cmd;
};
#endif

/**********************************************************************************/

struct katcp_message{
  unsigned int m_flags;
  struct katcl_parse *m_parse;
  struct katcp_endpoint *m_from;
  struct katcp_endpoint *m_to;
};

struct katcp_endpoint{
  unsigned int e_magic;
  unsigned short e_freeable;
  unsigned int e_state; 
  unsigned int e_refcount;

#if 0
  char *e_name;
  struct katcp_endpoint *e_peer;
#endif

  struct katcl_gueue *e_queue;
  unsigned int e_precedence;

  int (*e_wake)(struct katcp_dispatch *d, struct katcp_endpoint *ep, struct katcp_message *msg, void *data);
  void (*e_release)(struct katcp_dispatch *d, void *data);
  void *e_data;

  struct katcp_endpoint *e_next;
};

#if 0
#define KATCP_ENDPOINT_OWN    0x00
#define KATCP_ENDPOINT_FAIL   0x01
#define KATCP_ENDPOINT_OK     0x02
#define KATCP_ENDPOINT_QUICK  0x03
#define KATCP_ENDPOINT_STALL  0x04
#define KATCP_MASK_ENDPOINT   0x0f
#define KATCP_ENDPOINT_DEAD   0xf0
#endif

/**********************************************************************************/

void exchange_katcl(struct katcl_line *l, int fd);

int dispatch_cmd_katcp(struct katcp_dispatch *d, int argc);

void component_time_katcp(struct timeval *result, unsigned int ms);
int string_to_tv_katcp(struct timeval *tv, char *string);

int sub_time_katcp(struct timeval *delta, struct timeval *alpha, struct timeval *beta);
int add_time_katcp(struct timeval *sigma, struct timeval *alpha, struct timeval *beta);
int cmp_time_katcp(struct timeval *alpha, struct timeval *beta);


int startup_shared_katcp(struct katcp_dispatch *d);
void shutdown_shared_katcp(struct katcp_dispatch *d);
int listen_shared_katcp(struct katcp_dispatch *d, char *host, int port);
int allocate_clients_shared_katcp(struct katcp_dispatch *d, unsigned int count);
int link_shared_katcp(struct katcp_dispatch *d, struct katcp_dispatch *cd);

int load_shared_katcp(struct katcp_dispatch *d);
int run_shared_katcp(struct katcp_dispatch *d);
int ended_shared_katcp(struct katcp_dispatch *d);

void shutdown_cmd_katcp(struct katcp_cmd *c);

int define_cmd_katcp(struct katcp_dispatch *d, int argc);

/* sensor stuff */

int sensor_limit_cmd_katcp(struct katcp_dispatch *d, int argc);
int sensor_value_cmd_katcp(struct katcp_dispatch *d, int argc);
int sensor_list_cmd_katcp(struct katcp_dispatch *d, int argc);
int sensor_sampling_cmd_katcp(struct katcp_dispatch *d, int argc);
int sensor_dump_cmd_katcp(struct katcp_dispatch *d, int argc);
int sensor_cmd_katcp(struct katcp_dispatch *d, int argc);

int match_sensor_status_katcp(struct katcp_dispatch *d, struct katcp_notice *n, void *data);
int match_sensor_list_katcp(struct katcp_dispatch *d, struct katcp_notice *n, void *data);

/* misc */

char *code_to_name_katcm(int code);
char **copy_vector_katcm(char **vector, unsigned int size);
void delete_vector_katcm(char **vector, unsigned int size);
char *default_message_type_katcm(char *string, int type);

/* timing support */
int empty_timers_katcp(struct katcp_dispatch *d);
int run_timers_katcp(struct katcp_dispatch *d, struct timespec *interval);
void dump_timers_katcp(struct katcp_dispatch *d);

/* nonsense support */
void forget_nonsense_katcp(struct katcp_dispatch *d, unsigned int index);

/* how many times to try waitpid for child to exit */
#define KATCP_WAITPID_CHECKS 5 
/* how long to sleep between checks in nanoseconds */
#define KATCP_WAITPID_POLL   250000000UL

int term_signal_shared_katcp(struct katcp_shared *s);
int child_signal_shared_katcp(struct katcp_shared *s);

int reap_children_shared_katcp(struct katcp_dispatch *d, pid_t pid, int force);
int init_signals_shared_katcp(struct katcp_shared *s);
int undo_signals_shared_katcp(struct katcp_shared *s);

/* notice logic */
void disown_notices_katcp(struct katcp_dispatch *d);
void destroy_notices_katcp(struct katcp_dispatch *d);
int run_notices_katcp(struct katcp_dispatch *d);
int notice_cmd_katcp(struct katcp_dispatch *d, int argc);
int cancel_notice_katcp(struct katcp_dispatch *d, struct katcp_notice *n);

/* jobs */
int load_jobs_katcp(struct katcp_dispatch *d);
int wait_jobs_katcp(struct katcp_dispatch *d);
int run_jobs_katcp(struct katcp_dispatch *d);

int job_cmd_katcp(struct katcp_dispatch *d, int argc);
int register_subprocess_cmd_katcp(struct katcp_dispatch *d, int argc);
int submit_to_job_katcp(struct katcp_dispatch *d, struct katcp_job *j, struct katcl_parse *p, char *name, int (*call)(struct katcp_dispatch *d, struct katcp_notice *n, void *data), void *data);
int notice_to_job_katcp(struct katcp_dispatch *d, struct katcp_job *j, struct katcp_notice *n);
int ended_jobs_katcp(struct katcp_dispatch *d);

/* cmd stuff */

struct katcp_cmd_map *create_cmd_map_katcp(char *name);
void destroy_cmd_map_katcp(struct katcp_cmd_map *m);
void hold_cmd_map_katcp(struct katcp_cmd_map *m);
struct katcp_cmd_map *duplicate_cmd_map_katcp(struct katcp_cmd_map *mo, char *name);
struct katcp_cmd_item *find_cmd_map_katcp(struct katcp_cmd_map *m, char *name);
int remove_cmd_map_katcp(struct katcp_cmd_map *m, char *name);
void set_flag_cmd_item_katcp(struct katcp_cmd_item *ix, unsigned int flags);
int set_help_cmd_item_katcp(struct katcp_cmd_item *ix, char *help);

/* flat stuff */
int run_flat_katcp(struct katcp_dispatch *d);
int load_flat_katcp(struct katcp_dispatch *d);


/* duplex (flat+group) setup */
int startup_duplex_katcp(struct katcp_dispatch *d, unsigned int stories);
void shutdown_duplex_katcp(struct katcp_dispatch *d);

#define KATCP_GROUP_OVERRIDE_SENSOR    0x1000

int switch_group_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, struct katcp_group *gx);


#define KATCP_FLAT_CONNECTING   0x01   /* wait for connect to complete */
#define KATCP_FLAT_TOSERVER     0x02   /* direction */
#define KATCP_FLAT_TOCLIENT     0x04   /* direction */
#define KATCP_FLAT_HIDDEN       0x08   /* do not show up in client list */
#if 0
#define KATCP_FLAT_SEECHANGES   0x10   /* unused */
#endif
#define KATCP_FLAT_PREFIXED     0x20   /* sensors have prefix fields to them */
#define KATCP_FLAT_RETAINFO     0x40   /* do not rewrite relayed info fields */

struct katcp_flat *create_flat_katcp(struct katcp_dispatch *d, int fd, unsigned int flags, char *name, struct katcp_group *g);
struct katcp_flat *create_exec_flat_katcp(struct katcp_dispatch *d, unsigned int flags, char *name, struct katcp_group *gx, char **vector);
int reconfigure_flat_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, unsigned int flags);

int trigger_connect_flat(struct katcp_dispatch *d, struct katcp_flat *fx);

#define KATCP_ARB_TYPE_LISTENER 0xacce97

struct katcp_arb *create_listen_flat_katcp(struct katcp_dispatch *d, char *name, unsigned int port, char *address, struct katcp_group *g);
int destroy_listen_flat_katcp(struct katcp_dispatch *d, char *name);

/* parse: setup */
struct katcl_parse *create_parse_katcl();
struct katcl_parse *create_referenced_parse_katcl();
void destroy_parse_katcl(struct katcl_parse *p);
struct katcl_parse *reuse_parse_katcl(struct katcl_parse *p);
struct katcl_parse *copy_parse_katcl(struct katcl_parse *p);
struct katcl_parse *turnaround_parse_katcl(struct katcl_parse *p, int code);
struct katcl_parse *turnaround_extra_parse_katcl(struct katcl_parse *p, int code, char *fmt, ...);
struct katcl_parse *vturnaround_extra_parse_katcl(struct katcl_parse *p, int code, char *fmt, va_list args);

/* parse: adding fields */
int add_plain_parse_katcl(struct katcl_parse *p, int flags, char *string);
int add_string_parse_katcl(struct katcl_parse *p, int flags, char *buffer);
int add_unsigned_long_parse_katcl(struct katcl_parse *p, int flags, unsigned long v);
int add_signed_long_parse_katcl(struct katcl_parse *p, int flags, unsigned long v);
int add_hex_long_parse_katcl(struct katcl_parse *p, int flags, unsigned long v);
#ifdef KATCP_USE_FLOATS
int add_double_parse_katcl(struct katcl_parse *p, int flags, double v);
#endif
int add_buffer_parse_katcl(struct katcl_parse *p, int flags, void *buffer, unsigned int len);
int add_timestamp_parse_katcl(struct katcl_parse *p, int flags, struct timeval *tv);
int add_parameter_parse_katcl(struct katcl_parse *pd, int flags, struct katcl_parse *ps, unsigned int index);
int add_parameter_parse_katcl(struct katcl_parse *pd, int flags, struct katcl_parse *ps, unsigned int index);
int add_trailing_parse_katcl(struct katcl_parse *pd, int flags, struct katcl_parse *ps, unsigned int start);
int add_end_parse_katcl(struct katcl_parse *p);

int buffer_from_parse_katcl(struct katcl_parse *p, char *buffer, unsigned int len);

/* parse: extracting, testing fields */
unsigned int get_count_parse_katcl(struct katcl_parse *p);
int get_tag_parse_katcl(struct katcl_parse *p);

int is_type_parse_katcl(struct katcl_parse *p, char type);
int is_request_parse_katcl(struct katcl_parse *p);
int is_reply_parse_katcl(struct katcl_parse *p);
int is_inform_parse_katcl(struct katcl_parse *p);
int is_null_parse_katcl(struct katcl_parse *p, unsigned int index);

int is_reply_ok_parse_katcl(struct katcl_parse *p);

char *get_string_parse_katcl(struct katcl_parse *p, unsigned int index);
char *copy_string_parse_katcl(struct katcl_parse *p, unsigned int index);
unsigned long get_unsigned_long_parse_katcl(struct katcl_parse *p, unsigned int index);
long get_signed_long_parse_katcl(struct katcl_parse *p, unsigned int index);
int get_bb_parse_katcl(struct katcl_parse *p, unsigned int index, struct katcl_byte_bit *b);
#ifdef KATCP_USE_FLOATS
double get_double_parse_katcl(struct katcl_parse *p, unsigned int index);
#endif
unsigned int get_buffer_parse_katcl(struct katcl_parse *p, unsigned int index, void *buffer, unsigned int size);

/* parse: parsing from line */
int parse_katcl(struct katcl_line *l);
struct katcl_parse *ready_katcl(struct katcl_line *l);

int add_vargs_parse_katcl(struct katcl_parse *p, int flags, char *fmt, va_list args);
int add_args_parse_katcl(struct katcl_parse *p, int flags, char *fmt, ...);

#include <stdio.h>

int dump_parse_katcl(struct katcl_parse *p, char *prefix, FILE *fp);

/* */
int inform_client_connections_katcp(struct katcp_dispatch *d, char *type);

/* queue logic */
#if 0
struct katcl_parse *get_head_katcl(struct katcl_queue *q);
struct katcl_parse *get_tail_katcl(struct katcl_queue *q);
unsigned int is_empty_queue_katcl(struct katcl_queue *q);
#endif

struct katcl_parse *get_index_queue_katcl(struct katcl_queue *q, unsigned int index);

struct katcl_queue *create_queue_katcl(void);
void destroy_queue_katcl(struct katcl_queue *q);
void clear_queue_katcl(struct katcl_queue *q);
unsigned int size_queue_katcl(struct katcl_queue *q);

int add_tail_queue_katcl(struct katcl_queue *q, struct katcl_parse *p);
struct katcl_parse *remove_index_queue_katcl(struct katcl_queue *q, unsigned int index);
struct katcl_parse *remove_head_queue_katcl(struct katcl_queue *q);
struct katcl_parse *get_head_queue_katcl(struct katcl_queue *q);
void dump_queue_parse_katcp(struct katcl_queue *q, FILE *fp);

/* map logic */

struct katcp_map *create_map_katcp();
int destroy_map_katcp(struct katcp_dispatch *d, struct katcp_map *km, struct katcl_parse *p);
struct katcp_trap *find_map_katcp(struct katcp_map *km, char *name);
int remove_map_katcp(struct katcp_dispatch *d, struct katcp_map *km, char *name, struct katcl_parse *p);
int add_map_katcp(struct katcp_dispatch *d, struct katcp_map *km, char *name, struct katcp_notice *n);
int log_map_katcp(struct katcp_dispatch *d, char *prefix, struct katcp_map *km);

/* arbitrary filedescriptor callback support */

void load_arb_katcp(struct katcp_dispatch *d);
int arb_cmd_katcp(struct katcp_dispatch *d, int argc);
int run_arb_katcp(struct katcp_dispatch *d);
void destroy_arbs_katcp(struct katcp_dispatch *d);

/*katcp_type*/
void destroy_type_katcp(struct katcp_type *t);
struct katcp_type *create_type_katcp();
struct avl_tree *get_tree_type_katcp(struct katcp_type *t);
int binary_search_type_list_katcp(struct katcp_type **ts, int t_size, char *str);

int startup_services_katcp(struct katcp_dispatch *d);

struct katcp_dict {
  char *d_key;
  struct avl_tree *d_avl; 
};

struct katcp_dbase {
  char *d_key;
  char *d_schema;
  struct timeval d_stamped;
  struct katcp_stack *d_values;
};

struct katcp_tag {
  char *t_name;
  int t_level;

  void *t_tobject_root;
  int t_tobject_count;
};

void print_string_type_katcp(struct katcp_dispatch *d, char *key, void *data);
void destroy_string_type_katcp(void *data);
void *parse_string_type_katcp(struct katcp_dispatch *d, char **str);
char *getkey_dbase_type_katcp(void *data);
void print_dbase_type_katcp(struct katcp_dispatch *d, char *key, void *data);
void destroy_dbase_type_katcp(void *data);
void *parse_dbase_type_katcp(struct katcp_dispatch *d, char **str);
#if 1
void print_dict_type_katcp(struct katcp_dispatch *d, char *key, void *data);
void destroy_dict_type_katcp(void *data);
void *parse_dict_type_katcp(struct katcp_dispatch *d, char **str);
#endif

struct katcp_tag *create_tag_katcp(char *name, int level);
void destroy_tag_katcp(void *data);
void print_tag_katcp(struct katcp_dispatch *d, char *key, void *data);
void *parse_tag_katcp(struct katcp_dispatch *d, char **str);
int compare_tag_katcp(const void *m1, const void *m2);
char *getkey_tag_katcp(void *data);
int register_tag_katcp(struct katcp_dispatch *d, char *name, int level);


/* endpoints: internal ********************/

void run_endpoints_katcp(struct katcp_dispatch *d);
void load_endpoints_katcp(struct katcp_dispatch *d);

void release_endpoints_katcp(struct katcp_dispatch *d);

struct katcp_endpoint *create_endpoint_katcp(struct katcp_dispatch *d, int (*wake)(struct katcp_dispatch *d, struct katcp_endpoint *ep, struct katcp_message *msg, void *data), void (*release)(struct katcp_dispatch *d, void *data), void *data);

/* schedule destruction of endpoint, does not call release callback, to be called in destruction logic of entity owning the endpoint */
int release_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);

int flush_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);

void close_sending_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);
void close_receiving_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);

void show_endpoint_katcp(struct katcp_dispatch *d, char *prefix, int level, struct katcp_endpoint *ep);

int pending_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);


/* up reference count of endpoint, does not include reference to actual owner */
void reference_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);
/* decrement refcount */
void forget_endpoint_katcp(struct katcp_dispatch *d, struct katcp_endpoint *ep);

/* misc duplex functions ******************/

int code_from_scope_katcp(char *scope);
char *string_from_scope_katcp(unsigned int scope);

int fixup_timestamp_katcp(char *src, char *dst, int size);

/* internal variable use ******************/

#define KATCP_VRBL_DELIM_GROUP    '*'
#define KATCP_VRBL_DELIM_TREE     ':'
#define KATCP_VRBL_DELIM_ARRAY    '#' 
#define KATCP_VRBL_DELIM_LOGIC    '.'

#define KATCP_VRBL_DELIM_FORBID   '_'
#define KATCP_VRBL_DELIM_SPACER   '-'

struct katcp_region *create_region_katcp(struct katcp_dispatch *d);
void destroy_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx);

struct katcp_vrbl *find_vrbl_katcp(struct katcp_dispatch *d, char *key);
struct katcp_vrbl_payload *find_payload_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *path);

int type_payload_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
int find_type_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *path);
char *path_suffix_vrbl_katcp(struct katcp_dispatch *d, char *path);

struct katcp_vrbl *update_vrbl_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, char *name, struct katcp_vrbl *vo, int clobber);

int traverse_vrbl_katcp(struct katcp_dispatch *d, void *state, int (*callback)(struct katcp_dispatch *d, void *state, char *key, void *data));
int for_all_flats_vrbl_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, char *name, void *state, int (*callback)(struct katcp_dispatch *d, void *state, struct katcp_flat *fx));

int hide_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx);
int show_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx);

/* variable payload manipulation */

struct katcp_vrbl *scan_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *text, char *path, int create, unsigned int type);
#if 0
struct katcp_vrbl_payload *find_payload_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *path);
#endif

int add_payload_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
int configure_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, unsigned int flags, void *state, int (*refresh)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), int (*change)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), void (*release)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx));

void destroy_vrbl_katcp(struct katcp_dispatch *d, char *name, struct katcp_vrbl *vx);
/* variable type handling */

unsigned int type_from_string_vrbl_katcp(struct katcp_dispatch *d, char *string);
char *type_to_string_vrbl_katcp(struct katcp_dispatch *d, unsigned int type);
 
/* type specific top-level utilities */

struct katcp_vrbl *create_string_vrbl_katcp(struct katcp_dispatch *d, unsigned int flags, char *value);
int make_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_group *gx, char *key, unsigned int flags, char *value);

/* sensor specifics */

unsigned int current_strategy_sensor_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_flat *fx);

int is_vrbl_sensor_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx);

char *strategy_to_string_sensor_katcp(struct katcp_dispatch *d, unsigned int strategy);
int strategy_from_string_sensor_katcp(struct katcp_dispatch *d, char *name);

int monitor_event_variable_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_flat *fx);
int forget_event_variable_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_flat *fx);

struct katcl_parse *make_sensor_katcp(struct katcp_dispatch *d, char *name, struct katcp_vrbl *vx, char *prefix);

#define KATCP_NAGLE_CHANGE  500000    /* defer device-changes by this much (us) ... */
int schedule_sensor_update_katcp(struct katcp_dispatch *d, char *name);


/* version callback */

int version_generic_callback_katcp(struct katcp_dispatch *d, void *state, char *key, struct katcp_vrbl *vx);

/******************************************/

int prepend_generic_flat_katcp(struct katcp_dispatch *d, int reply);

/******************************************/

#define KATCL_PARSE_MAGIC 0xff7f1273

#define KATCP_PRINT_VERSION_CONNECT  0
#define KATCP_PRINT_VERSION_LIST     1
#define KATCP_PRINT_VERSION          2 /* deprecated as of V5 */

#ifdef __cplusplus
}
#endif

#endif
