#ifdef KATCP_EXPERIMENTAL

/* The user interface to operate on variables is (still) inconsistent:
 *
 * Printing a full compound variable can be done, but
 * setup allows only element at a time
 *
 * Printing ommits keys, etc of compound variables, setup requires them ... 
 *
 * Setup guesses type of individual interior
 * elements, from path specifier 
 *
 */ 

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <ctype.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sysexits.h>

#include <sys/stat.h>

#include <katcp.h>
#include <katpriv.h>
#include <katcl.h>
#include <avltree.h>

#define MAX_DEPTH_VRBL 3
#define MAX_COUNT_STAR 3

static struct katcp_vrbl *find_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, char *key);
static int insert_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, struct katcp_vrbl *vx, char *key);
static int remove_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, struct katcp_vrbl *vx);

/**********************************************************************************/

void release_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
struct katcp_vrbl_payload *setup_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, unsigned int type);

int add_payload_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);

/* lookup structure for all types *************************************************/

struct katcp_vrbl_type_ops{
  char *t_name;

  /* set up union, assumes garbled input - produces usable union/structure with type set, with some resources allocated possibly */
  int (*t_init)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
  /* releases all resources held in union */
  void (*t_clear)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);


  /* WARNING unclear semantics: how do we build up a composite structure ..., add and scan should be complementary ?  */
  int (*t_scan)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type);

  /* retrieve a particular element ... */
  struct katcp_vrbl_payload *(*t_element)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path);

  /* appends content to px, can recurse */
  int (*t_add)(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);

#if 0
  int (*t_traverse)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, int flags, int (*call)(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path));
#endif
};

int init_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
void clear_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
int scan_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type);
struct katcp_vrbl_payload *element_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path);
int add_string_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);

int init_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
void clear_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
int scan_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type);
struct katcp_vrbl_payload *element_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path);
int add_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);

int init_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
void clear_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);
int scan_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type);
struct katcp_vrbl_payload *element_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path);
int add_array_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py);

struct katcp_vrbl_type_ops ops_type_vrbl[KATCP_MAX_VRT] = {
  [KATCP_VRT_STRING] = 
  { "string",
    &init_string_vrbl_katcp,
    &clear_string_vrbl_katcp,
    &scan_string_vrbl_katcp,
    &element_string_vrbl_katcp,
    &add_string_vrbl_katcp
  }, 
  [ KATCP_VRT_TREE] = 
  { "map",
    &init_tree_vrbl_katcp,
    &clear_tree_vrbl_katcp,
    &scan_tree_vrbl_katcp,
    &element_tree_vrbl_katcp,
    &add_tree_vrbl_katcp
  },
  [ KATCP_VRT_ARRAY] = 
  { "array",
    &init_array_vrbl_katcp,
    &clear_array_vrbl_katcp,
    &scan_array_vrbl_katcp,
    &element_array_vrbl_katcp,
    &add_array_vrbl_katcp
  }
};

/* utils **************************************************************************/

static int next_element_path_vrbl(char *path)
{
  int i;

  if(path == NULL){
    return -1;
  }

  for(i = 0; path[i] != '\0'; i++){
    switch(path[i]){
      case KATCP_VRBL_DELIM_TREE : 
      case KATCP_VRBL_DELIM_ARRAY :
#if 0
      case KATCP_VRBL_DELIM_LOGIC : 
#endif
        if(i > 1){
          return i;
        } else if(i == 1){
          return -1;
        }
        break;
      case '\n' :
      case '\r' :
        return -1;
    }
  }

  if(i <= 1){
    return -1;
  }

  /* no more ? */

  return 0;
}

static unsigned int infer_type_path_vrbl(char *path)
{
  if(path == NULL){
    return KATCP_VRT_GONE;
  }

  switch(path[0]){
    case KATCP_VRBL_DELIM_TREE : 
      return KATCP_VRT_TREE;
    case KATCP_VRBL_DELIM_ARRAY :
      return KATCP_VRT_ARRAY;
#if 0
    case KATCP_VRBL_DELIM_LOGIC : 
      return TODO;
#endif
      /* case '\0' : */
    default : 
      return KATCP_VRT_GONE;
  }
}

int fixup_type_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, unsigned int type)
{
  int old;

  if(type >= KATCP_MAX_VRT){
    return -1;
  }

  if((py == NULL) || (vx == NULL)){
    return -1;
  }

  if(py->p_type == type){
    return 0;
  }

  if(vx->v_flags & KATCP_VRF_FLX){
    old = py->p_type;
  } else {
    return -1;
  }

  if(old < KATCP_MAX_VRT){
    (*(ops_type_vrbl[old].t_clear))(d, vx, py);
  }

  py->p_type = KATCP_VRT_GONE;

  if((*(ops_type_vrbl[type].t_init))(d, vx, py) < 0){
    return -1;
  }

  return 1;
}

static struct katcp_vrbl *allocate_vrbl_katcp(struct katcp_dispatch *d, unsigned int type)
{
  struct katcp_vrbl *vx;

  if(type >= KATCP_MAX_VRT){
    /* WARNING - this can not be called bare */
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unwilling to create an untyped variable");
    return NULL;
  }

  vx = malloc(sizeof(struct katcp_vrbl));
  if(vx == NULL){
    return NULL;
  }

  vx->v_name = NULL;

  vx->v_status = 0;
  vx->v_flags = 0;

  vx->v_refresh = NULL;
  vx->v_change = NULL;
  vx->v_release = NULL;

  vx->v_payload = setup_vrbl_katcp(d, vx, type);
  if(vx->v_payload == NULL){
    free(vx);
    return NULL;
  }

  vx->v_extra = NULL;

  return vx;
}

/* string logic *******************************************************************/

int init_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  if(py == NULL){
    return -1;
  }

  if(py->p_type != KATCP_VRT_GONE){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "variable: attempting to initialise a variable of type %u which is already set up to be %u\n", KATCP_VRT_STRING, py->p_type);
    abort();
#else
    return -1;
#endif
  }

  py->p_type = KATCP_VRT_STRING;
  py->p_union.u_string = NULL;

  return 0;
}

void clear_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
#ifdef KATCP_CONSISTENCY_CHECKS
  if(py == NULL){
    fprintf(stderr, "clear string: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_STRING){
    fprintf(stderr, "clear string: bad type %u\n", py->p_type);
    abort();
  }
#endif

  if(py->p_union.u_string){
    free(py->p_union.u_string);
    py->p_union.u_string = NULL;
  }

  py->p_type = KATCP_VRT_GONE;
}

int set_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *value)
{
  char *ptr;
  int len;
  struct katcp_vrbl_payload *ty;

  if(vx == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
    abort();
#else
    return -1;
#endif
  }

  if(py == NULL){
    /* WARNING: null assumes the default ... could be tricky if a complicated structure */
    ty = vx->v_payload;
    if(ty == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
      fprintf(stderr, "string variable: no payload available for variable value %s\n", value);
      abort();
#endif
      return -1;
    }
  } else {
    ty = py;
  }

  if(fixup_type_vrbl_katcp(d, vx, ty, KATCP_VRT_STRING) < 0){
    return -1;
  }

  if(value == NULL){
    if(ty->p_union.u_string){
      free(ty->p_union.u_string);
      ty->p_union.u_string = NULL;
    }
    return 0;
  }

  len = strlen(value) + 1;
  ptr = realloc(ty->p_union.u_string, len);
  if(ptr == NULL){
    return -1;
  }

  memcpy(ptr, value, len);
  ty->p_union.u_string = ptr;
  return 0;
}

int scan_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type)
{

  if(text){
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to scan string %s", text);
  } else {
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to initialise string");
  }

  if((path != NULL) && (path[0] != '\0')){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "compound variable access path %s for string type", path);
    return -1;
  }

  /* special case, as scan uses a string as input too, other types not so easy */
  return set_string_vrbl_katcp(d, vx, py, text);
}

struct katcp_vrbl_payload *element_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path)
{
  if((path == NULL) || (path[0] == '\0')){
    return py;
  }

  return NULL;
}

int add_string_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  char *ptr;
  unsigned int mask;

#ifdef KATCP_CONSISTENCY_CHECKS
  if(py == NULL){
    fprintf(stderr, "add variable string: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_STRING){
    fprintf(stderr, "add variable string: bad type %u\n", py->p_type);
    abort();
  }
#endif

  mask = (KATCP_FLAG_FIRST | KATCP_FLAG_LAST) & flags;
  ptr = py->p_union.u_string;

  return add_string_parse_katcl(px, mask | KATCP_FLAG_STRING, ptr);
}

/* tree logic *******************************************************************/

int init_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  if(py == NULL){
    return -1;
  }

  if(py->p_type != KATCP_VRT_GONE){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "variable: attempting to initialise a variable of type %u which is already set up to be %u\n", KATCP_VRT_TREE, py->p_type);
    abort();
#else
    return -1;
#endif
  }

  py->p_union.u_tree = create_avltree();

  if(py->p_union.u_tree == NULL){
    return -1;
  }

  py->p_type = KATCP_VRT_TREE;

  return 0;
}

struct clear_tree_variable_state{
  struct katcp_dispatch *s_dispatch;
  struct katcp_vrbl *s_variable;
};

static void callback_clear_tree_vrbl(void *global, char *key, void *payload)
{
  struct clear_tree_variable_state *st;
  struct katcp_vrbl_payload *py;

  py = payload;
  st = global;

  release_vrbl_katcp(st->s_dispatch, st->s_variable, py);
}

void clear_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  struct avl_tree *tree;
  struct clear_tree_variable_state state, *st;

#ifdef KATCP_CONSISTENCY_CHECKS
  if(py == NULL){
    fprintf(stderr, "clear tree: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_TREE){
    fprintf(stderr, "clear tree: bad type %u\n", py->p_type);
    abort();
  }
#endif

  tree = py->p_union.u_tree;

  if(tree){
    st = &state;

    st->s_dispatch = d;
    st->s_variable = vx;

    destroy_complex_avltree(tree, st, &callback_clear_tree_vrbl);

    py->p_union.u_tree = NULL;
  }

  py->p_type = KATCP_VRT_GONE;
}

#if 0
int set_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *value)
{
  char *ptr;
  int len;
  struct katcp_vrbl_payload *ty;

  if(vx == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
    abort();
#else
    return -1;
#endif
  }

  if(py == NULL){
    /* WARNING: null assumes the default ... could be tricky if a complicated structure */
    ty = vx->v_payload;
  } else {
    ty = py;
  }

  if(ty->p_type != KATCP_VRT_STRING){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "string variable: bad type %u\n", ty->p_type);
    abort();
#else
    return -1;
#endif
  }

  if(value == NULL){
    if(ty->p_union.u_string){
      free(ty->p_union.u_string);
      ty->p_union.u_string = NULL;
    }
    return 0;
  }

  len = strlen(value) + 1;
  ptr = realloc(ty->p_union.u_string, len);
  if(ptr == NULL){
    return -1;
  }

  memcpy(ptr, value, len);
  ty->p_union.u_string = ptr;
  return 0;
}

#endif

int scan_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type)
{
  int next, len, result;
  unsigned int infer;
  char *ptr, *copy, *rest;
  struct katcp_vrbl_payload *ty;
  struct avl_tree *tree;

  if(path){
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to scan map with key %s", path);
  } else {
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to scan empty map");
  }

  if(fixup_type_vrbl_katcp(d, vx, py, KATCP_VRT_TREE) < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable fails type consistency rule");
    return -1;
  }

  tree = py->p_union.u_tree;

  if(path == NULL){
    if(text == NULL){
      log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "scan of empty map");
      return 0;
    } else {
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "map requires a path for value insertion");
      return -1;
    }
  }

  if(path[0] != KATCP_VRBL_DELIM_TREE){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "tree insert: unexpected path specification %s\n", path);
#endif
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid compound variable access path %s", path);
    return -1;
  }

  next = next_element_path_vrbl(path);
  if(next < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid path specification %s", path);
    return -1;
  }

  if(next == 0){
    ptr = path + 1;
    copy = NULL;
    rest = NULL;
  } else {
    if(next == 1){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "compound variable access path has a null member");
      return -1;
    }
    len = next;
    
    copy = malloc(len + 1);
    if(copy == NULL){
      return -1;
    }
    memcpy(copy, path + 1, len);
    copy[len - 1] = '\0';

    ptr = copy;
    rest = path + next;
  }

  ty = find_data_avltree(tree, ptr);
  if(ty == NULL){

    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "tried to locate element %s but not found", ptr);

    if((create == 0) || (type >= KATCP_MAX_VRT)){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable element %s would have to be created, but creation not requested", ptr);
      if(copy){
        free(copy);
      }
      return -1;
    }

    infer = infer_type_path_vrbl(rest);

    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "inferred type %u with leaf %u", infer, type);

    if(infer >= KATCP_MAX_VRT){
      infer = type;
    }

    ty = setup_vrbl_katcp(d, vx, infer);
    if(ty == NULL){
      if(copy){
        free(copy);
      }
      return -1;
    }

    if(store_named_node_avltree(tree, ptr, ty)){
      release_vrbl_katcp(d, vx, ty);
      if(copy){
        free(copy);
      }
      return -1;
    }
  } else {
    infer = ty->p_type;
#ifdef KATCP_CONSISTENCY_CHECKS
    if(infer >= KATCP_MAX_VRT){
      fprintf(stderr, "tree insert: operating on bad type %u\n", infer);
    }
#endif
  }

  if(copy){
    free(copy);
  }

  /* go deeper down the wabbit hole */
  result = (*(ops_type_vrbl[infer].t_scan))(d, vx, ty, text, rest, create, type);
  
  if(result < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "child addition to map failed");
  }

#if 0
  if(vx->v_change){
    /* WARNING: change API update happens at top level, assuming type_specific scans are only called at proper entry points */
  }
#endif

  return result;
}

struct katcp_vrbl_payload *element_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path)
{
  struct avl_tree *tree;
  int next, len;
  unsigned int infer;
  struct katcp_vrbl_payload *ty;
  char *ptr, *copy, *rest;

  if((vx == NULL) || (py == NULL)){
    return NULL;
  }

  if((path == NULL) || (path[0] == '\0')){
    return py;
  }

  /* WARNING: this has a fair bit in common with the scan logic, but splitting it out is messy ... */

  if(path[0] != KATCP_VRBL_DELIM_TREE){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid compound variable access path %s at tree", path);
    return NULL;
  }

  next = next_element_path_vrbl(path);
  if(next < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid path specification %s", path);
    return NULL;
  }

  if(next == 0){
    ptr = path + 1;
    copy = NULL;
    rest = NULL;
  } else {
    if(next == 1){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "compound variable access path has a null member");
      return NULL;
    }
    len = next;
    
    copy = malloc(len + 1);
    if(copy == NULL){
      return NULL;
    }
    memcpy(copy, path + 1, len);
    copy[len - 1] = '\0';

    ptr = copy;
    rest = path + next;
  }

  tree = py->p_union.u_tree;
#ifdef KATCP_CONSISTENCY_CHECKS
  if(tree == NULL){
    fprintf(stderr, "major logic problem: variable payload is supposed to be a tree, but is null\n");
    abort();
  }
#endif

  ty = find_data_avltree(tree, ptr);

  if(copy){
    free(copy);
  }

  if(ty == NULL){
    return NULL;
  }

  infer = ty->p_type;
#ifdef KATCP_CONSISTENCY_CHECKS
  if(infer >= KATCP_MAX_VRT){
    fprintf(stderr, "major logic problem: looked up entity is invalid type\n");
    abort();
  }
#endif

  /* go deeper down the wabbit hole */
  return (*(ops_type_vrbl[infer].t_element))(d, vx, ty, rest);
}

struct add_tree_variable_state{
  struct katcl_parse *s_parse;
  struct katcp_vrbl *s_variable;
  int s_flags;
  int s_result;
};

static int callback_add_tree_vrbl(struct katcp_dispatch *d, void *global, char *key, void *data)
{
  int result;
  struct katcp_vrbl_payload *py;
  struct add_tree_variable_state *st;

  if((key == NULL) || (data == NULL)){
    return -1;
  }

  py = data;
  st = global;

  result = add_payload_vrbl_katcp(d, st->s_parse, st->s_flags, st->s_variable, py);

  if((result >= 0) && (st->s_result >= 0)){
    st->s_result += result;
  } else {
    st->s_result = (-1);
  }

  st->s_flags = st->s_flags & ~(KATCP_FLAG_FIRST);

  return 0;
}

int add_tree_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  /* WARNING: trees can be empty, this will force output of a \@ if this is the case and no adornments have been specified */
  struct avl_tree *tree;
  struct add_tree_variable_state state, *st;
  int sum, result;

#ifdef KATCP_CONSISTENCY_CHECKS
  unsigned int valid;

  if(py == NULL){
    fprintf(stderr, "add tree: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_TREE){
    fprintf(stderr, "add tree: bad type %u\n", py->p_type);
    abort();
  }
  valid = (KATCP_FLAG_FIRST | KATCP_FLAG_LAST | KATCP_FLAG_PRETTY);
  if((flags | valid) != valid){
    fprintf(stderr, "add tree: bad flags 0x%x supplied\n", flags);
    abort();
  }
#endif

  st = &state;

  sum = 0;

  st->s_result = 0;
  st->s_parse = px;
  st->s_variable = vx;
  st->s_flags = flags & KATCP_FLAG_PRETTY;

  if(flags & KATCP_FLAG_PRETTY){
    result = add_string_parse_katcl(px, (flags & KATCP_FLAG_FIRST) | KATCP_FLAG_STRING, "{");
    if(result < 0){
      return -1;
    }
    sum += result;
  } else {
    st->s_flags |= (flags & KATCP_FLAG_FIRST);
  }

  tree = py->p_union.u_tree;

  complex_inorder_traverse_avltree(d, tree->t_root, st, &callback_add_tree_vrbl);
  if(st->s_result < 0){
    return -1;
  }
  sum += st->s_result;

  if(flags & KATCP_FLAG_PRETTY){
    result = add_string_parse_katcl(px, (flags & KATCP_FLAG_LAST) | KATCP_FLAG_STRING, "}");
    if(result < 0){
      return -1;
    }
    sum += result;
  } else {
    if((flags & KATCP_FLAG_FIRST) && (st->s_flags & KATCP_FLAG_FIRST)){
      result = add_string_parse_katcl(px, KATCP_FLAG_FIRST | KATCP_FLAG_STRING, NULL);
      if(result < 0){
        return -1;
      }
      sum += result;
    }
    if(flags & KATCP_FLAG_LAST){
      /* WARNING: finish manually, instead of using last flag */
      add_end_parse_katcl(px);
    }
  }

  return sum;
}

/* array logic *******************************************************************/

int init_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  struct katcp_vrbl_array *va;

  if(py == NULL){
    return -1;
  }

  if(py->p_type != KATCP_VRT_GONE){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "variable: attempting to initialise a variable of type %u which is already set up to be %u\n", KATCP_VRT_ARRAY, py->p_type);
    abort();
#else
    return -1;
#endif
  }

  py->p_type = KATCP_VRT_ARRAY;

  va = &(py->p_union.u_array);

  va->a_size = 0;
  va->a_elements = NULL;

  return 0;
}

void clear_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  struct katcp_vrbl_array *va;
  unsigned int i;

#ifdef KATCP_CONSISTENCY_CHECKS
  if(py == NULL){
    fprintf(stderr, "clear array: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_ARRAY){
    fprintf(stderr, "clear array: bad type %u\n", py->p_type);
    abort();
  }
#endif

  va = &(py->p_union.u_array);

  if(va->a_elements){
    for(i = 0; i < va->a_size; i++){
      release_vrbl_katcp(d, vx, va->a_elements[i]);
      va->a_elements[i] = NULL;
    }
    free(va->a_elements);
    va->a_elements = NULL;
  }

  va->a_size = 0;

  py->p_type = KATCP_VRT_GONE;
}

#if 0
int set_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *value)
{
  char *ptr;
  int len;
  struct katcp_vrbl_payload *ty;

  if(vx == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
    abort();
#else
    return -1;
#endif
  }

  if(py == NULL){
    /* WARNING: null assumes the default ... could be tricky if a complicated structure */
    ty = vx->v_payload;
    if(ty == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
      fprintf(stderr, "string variable: no payload available for variable value %s\n", value);
      abort();
#endif
      return -1;
    }
  } else {
    ty = py;
  }

  if(fixup_type_vrbl_katcp(d, vx, ty, KATCP_VRT_STRING) < 0){
    return -1;
  }

  if(value == NULL){
    if(ty->p_union.u_string){
      free(ty->p_union.u_string);
      ty->p_union.u_string = NULL;
    }
    return 0;
  }

  len = strlen(value) + 1;
  ptr = realloc(ty->p_union.u_string, len);
  if(ptr == NULL){
    return -1;
  }

  memcpy(ptr, value, len);
  ty->p_union.u_string = ptr;
  return 0;
}
#endif

int scan_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *text, char *path, int create, unsigned int type)
{
  struct katcp_vrbl_array *va;
  unsigned int index, i;
  int next, result;
  unsigned int infer;
  char *rest, *end;
  struct katcp_vrbl_payload *ty;
  struct katcp_vrbl_payload **tmp;

  if(path){
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to scan map with key %s", path);
  } else {
    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "about to scan empty map");
  }

  if(fixup_type_vrbl_katcp(d, vx, py, KATCP_VRT_ARRAY) < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable fails type consistency rule");
    return -1;
  }

  va = &(py->p_union.u_array);

  if(path == NULL){
    if(text == NULL){
      log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "scan of empty array");
      return 0;
    } else {
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "array requires a path for value insertion");
      return -1;
    }
  }

  if(path[0] != KATCP_VRBL_DELIM_ARRAY){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "tree insert: unexpected path specification %s\n", path);
#endif
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid compound variable access path %s", path);
    return -1;
  }

  next = next_element_path_vrbl(path);
  if(next < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid path specification %s", path);
    return -1;
  }

  index = strtoul(path + 1, &end, 10);
  if(end == (path + 1)){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "no index available in path specification %s", path);
    return -1;
  }

  if(isalpha(end[0])){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "incorrect suffix %s while scaning path specification %s", end, path);
    return -1;
  }

  if(next == 1){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "compound variable access path has a null member");
    return -1;
  }

  if(next == 0){
    rest = NULL;
  } else {
    rest = path + next;
  }

  if(va->a_size <= index){
    if((create == 0) || (type >= KATCP_MAX_VRT)){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable element %s would have to be created, but creation not requested", path);
      return -1;
    }

    tmp = realloc(va->a_elements, sizeof(struct katcp_vrbl_payload *) * (index + 1));
    if(tmp == NULL){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "unable to resize array to %u elements", index);
      return -1;
    }

    va->a_elements = tmp;

    log_message_katcp(d, KATCP_LEVEL_TRACE, NULL, "resized vector from %u to %u", va->a_size, index + 1);

    for(i = va->a_size; i <= index; i++){
      va->a_elements[i] = NULL;
    }

    va->a_size = (index + 1);
  }

  ty = va->a_elements[index];

  if(ty == NULL){

    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "tried to locate element %s but not found", path);

    if((create == 0) || (type >= KATCP_MAX_VRT)){
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable element %s would have to be created, but creation not requested", path);
      return -1;
    }

    infer = infer_type_path_vrbl(rest);

    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "inferred type %u with leaf %u", infer, type);

    if(infer >= KATCP_MAX_VRT){
      infer = type;
    }

    ty = setup_vrbl_katcp(d, vx, infer);
    if(ty == NULL){
      return -1;
    }

    va->a_elements[index] = ty;

  } else {
    infer = ty->p_type;
#ifdef KATCP_CONSISTENCY_CHECKS
    if(infer >= KATCP_MAX_VRT){
      fprintf(stderr, "tree insert: operating on bad type %u\n", infer);
    }
#endif
  }

  /* go deeper down the wabbit hole */
  result = (*(ops_type_vrbl[infer].t_scan))(d, vx, ty, text, rest, create, type);
  
  if(result < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "child addition to map failed");
  }

#if 0
  if(vx->v_change){
    /* WARNING: change API update happens at top level, assuming type_specific scans are only called at proper entry points */
  }
#endif

  return result;
}

struct katcp_vrbl_payload *element_array_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py, char *path)
{
  struct katcp_vrbl_array *va;
  struct katcp_vrbl_payload *ty;
  unsigned int infer, index;
  int next;
  char *rest, *end;

  if((vx == NULL) || (py == NULL)){
    return NULL;
  }

  if((path == NULL) || (path[0] == '\0')){
    return py;
  }

  /* WARNING: this has a fair bit in common with the scan logic, but splitting it out is messy ... */

  if(path[0] != KATCP_VRBL_DELIM_ARRAY){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid compound variable access path %s at array", path);
    return NULL;
  }

  next = next_element_path_vrbl(path);
  if(next < 0){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "invalid path specification %s", path);
    return NULL;
  }

  index = strtoul(path + 1, &end, 10);
  if(end == (path + 1)){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "no index available in path specification %s", path);
    return NULL;
  }

  if(isalpha(end[0])){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "incorrect suffix %s while scaning path specification %s", end, path);
    return NULL;
  }

  switch(next){
    case 0 :
      rest = NULL;
      break;
    case 1 : 
      log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "compound variable access path has a null member");
      return NULL;
    default : 
      rest = path + next;
      break;
  }

  va = &(py->p_union.u_array);

  if(va->a_size <= index){
    return NULL;
  } 


  ty = va->a_elements[index];
  if(ty == NULL){
    return NULL;
  }

  infer = ty->p_type;
#ifdef KATCP_CONSISTENCY_CHECKS
  if(infer >= KATCP_MAX_VRT){
    fprintf(stderr, "major logic problem: looked up entity is invalid type\n");
    abort();
  }
#endif

  /* go deeper down the wabbit hole */
  return (*(ops_type_vrbl[infer].t_element))(d, vx, ty, rest);
}

int add_array_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  /* WARNING: arrays can also be empty, this will force output of a \@ if this is the case and no adornments have been specified */
  struct katcp_vrbl_array *va;
  unsigned int i, elements;
  int result, sum, f;

#ifdef KATCP_CONSISTENCY_CHECKS
  unsigned int valid;

  if(py == NULL){
    fprintf(stderr, "add variable array: no variable given\n");
    abort();
  }
  if(py->p_type != KATCP_VRT_ARRAY){
    fprintf(stderr, "add variable array: bad type %u\n", py->p_type);
    abort();
  }
  valid = (KATCP_FLAG_FIRST | KATCP_FLAG_LAST | KATCP_FLAG_PRETTY);
  if((flags | valid) != valid){
    fprintf(stderr, "add variable array: bad flags 0x%x supplied\n", flags);
    abort();
  }
#endif

  sum = 0;
  elements = 0;
  f = flags & KATCP_FLAG_PRETTY;

  if(flags & KATCP_FLAG_PRETTY){
    result = add_string_parse_katcl(px, (flags & KATCP_FLAG_FIRST) | KATCP_FLAG_STRING, "[");
    if(result < 0){
      return -1;
    }
    sum += result;
  } else {
    f |= (flags & KATCP_FLAG_FIRST);
  }

  va = &(py->p_union.u_array);

  for(i = 0; i < va->a_size; i++){
    if(va->a_elements[i]){
      result = add_payload_vrbl_katcp(d, px, f, vx, va->a_elements[i]);
      if(result < 0){
        return -1;
      }
      elements++;
      sum += result;
      f &= ~KATCP_FLAG_FIRST;
    }
  }

  if(flags & KATCP_FLAG_PRETTY){
    result = add_string_parse_katcl(px, (flags & KATCP_FLAG_LAST) | KATCP_FLAG_STRING, "]");
    if(result < 0){
      return -1;
    }
    sum += result;
  } else {
    if((flags & KATCP_FLAG_FIRST) && (elements <= 0)){
      result = add_string_parse_katcl(px, KATCP_FLAG_FIRST | KATCP_FLAG_STRING, NULL);
      if(result < 0){
        return -1;
      }
      sum += result;
    }
    if(flags & KATCP_FLAG_LAST){
      /* WARNING: finish manually, instead of using last flag */
      add_end_parse_katcl(px);
    }
  }

  return sum;
}

/* type name related functions ****************************************************/

unsigned int type_from_string_vrbl_katcp(struct katcp_dispatch *d, char *string)
{
  int i;

  if(string == NULL){
    return KATCP_VRT_GONE;
  }

  for(i = 0; i < KATCP_MAX_VRT; i++){
    if(!strcmp(string, ops_type_vrbl[i].t_name)){
      return i;
    }
  }

  return KATCP_VRT_GONE;
}

char *type_to_string_vrbl_katcp(struct katcp_dispatch *d, unsigned int type)
{
  if(type >= KATCP_MAX_VRT){
    return NULL;
  }

  return ops_type_vrbl[type].t_name;
}

/* WARNING: order important, needs to correspond to bit position */

static char *flag_lookup_vrbl[] = { "environment",  "version", "sensor", "fluid", "hidden", NULL };

unsigned int flag_from_string_vrbl_katcp(struct katcp_dispatch *d, char *string)
{
  int i;

  if(string == NULL){
    return 0;
  }

  for(i = 0; flag_lookup_vrbl[i]; i++){
    if(!strcmp(flag_lookup_vrbl[i], string)){
      return 1 << i;
    }
  }

  return 0;
}

char *flag_to_string_vrbl_katcp(struct katcp_dispatch *d, unsigned int flag)
{
  unsigned int i, v;

  if(flag <= 0){
    return NULL;
  }

  v = flag;

  for(i = 0; flag_lookup_vrbl[i]; i++){
    if(v == 1){
      return flag_lookup_vrbl[i];
    }
    v = v >> 1;
  }

  return NULL;
}

/* payload setup/destroy **********************************************************/

struct katcp_vrbl_payload *setup_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, unsigned int type)
{
  struct katcp_vrbl_payload *py;

  if(type >= KATCP_MAX_VRT){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "logic problem while creating a variable: unknown type %u\n", type);
    abort();
#else
    return NULL;
#endif
  }

  py = malloc(sizeof(struct katcp_vrbl_payload));
  if(py == NULL){
    return NULL;
  }

  py->p_type = KATCP_VRT_GONE;

  if((*(ops_type_vrbl[type].t_init))(d, vx, py) < 0){
    free(py);
    return NULL;
  }

  return py;
}

void release_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  if(py == NULL){
    return;
  }

  if(py->p_type < KATCP_MAX_VRT){
    (*(ops_type_vrbl[py->p_type].t_clear))(d, vx, py);
    py->p_type = KATCP_VRT_GONE;
#ifdef KATCP_CONSISTENCY_CHECKS
  } else if(py->p_type == KATCP_VRT_GONE){
    fprintf(stderr, "logic failure in variable destruction: variable %p already gone, probably because of some prior allocation failure\n", py);
  } else {
    fprintf(stderr, "major logic failure: variable payload %p has an unknown type %u\n", py, py->p_type);
    abort();
#endif
  }

  free(py);
}

/* generic functions using type ops ***********************************************/

struct katcp_vrbl *scan_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *text, char *path, int create, unsigned int type)
{
  int result;
  struct katcp_vrbl_payload *py;
  struct katcp_vrbl *vt;
  unsigned int infer;

  if(path){
    infer = infer_type_path_vrbl(path);
    if(infer >= KATCP_MAX_VRT){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "invalid path %s for variable", path);
      return NULL;
    }
  } else {
    infer = type;
  }

  if(vx == NULL){
    if(create == 0){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "requested to create a new variable without create flag being set");
      return NULL;
    } 

    vt = allocate_vrbl_katcp(d, infer);
    if(vt == NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "variable allocation failure");
      return NULL;
    }

  } else {
    vt = vx;
  }

  py = vt->v_payload;

  if(py->p_type >= KATCP_MAX_VRT){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "invalid variable type");
    return NULL;
  }

  result = (*(ops_type_vrbl[py->p_type].t_scan))(d, vt, py, text, path, create, type);
  if(result < 0){

    log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "unable to scan variable %s", text ? text : "[null]");

    if(vx == NULL){
      /* we created it, we'd better clean up too */
      release_vrbl_katcp(d, vt, py);
      free(vt);
    }

    return NULL;
  }

  if(vt->v_change){
    /* WARNING: unclear if this is supposed to happen this far down the API ? */
    /* also unclear if this needs to trigger on an unsuccessful update */
    (*(vt->v_change))(d, vt->v_extra, vt->v_name, vt);
  }
  
  return vt;
}

int flags_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, unsigned int clear, unsigned int set)
{
  if(vx == NULL){
    return -1;
  }

  vx->v_flags = (vx->v_flags & (clear ^ KATCP_MASK_VRF)) | set;

  return 0;
}

int hide_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx)
{
  return flags_vrbl_katcp(d, vx, 0, KATCP_VRF_HID);
}

int show_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx)
{
  return flags_vrbl_katcp(d, vx, KATCP_VRF_HID, 0);
}

int configure_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, unsigned int flags, void *state, int (*refresh)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), int (*change)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), void (*release)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx))
{
  if(vx == NULL){
    return -1;
  }

  /* WARNING: configure gets used by sensors ... risk of clobbering things */

  if(vx->v_extra){
    log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "variable %s has already been given local state thus refusing configuration", vx->v_name);
    return -1;
  }

  vx->v_flags = flags;
  /* WARNING what about status ? */

  vx->v_refresh = refresh;
  vx->v_change = change;
  vx->v_release = release;

  vx->v_extra = state;

  return 0;
}

char *path_suffix_vrbl_katcp(struct katcp_dispatch *d, char *path)
{
  unsigned int infer;
  int next;

  if(path){
    infer = infer_type_path_vrbl(path);
    if(infer >= KATCP_MAX_VRT){
      next = next_element_path_vrbl(path);
      if(next > 0){ /* didn't look like the start of a path, but has more down the line -> assume region prefix, remove it */
        return path + next;
      } else {
        return NULL; /* didn't look like the start of a path, nothing down the line -> assume string, remove all */
      }
    } else {
      return path; /* looks like the start of a valid path */
    }
  } else {
    return NULL; /* no choice, null path */
  }
}

struct katcp_vrbl_payload *find_payload_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *path)
{
  unsigned int type;
  struct katcp_vrbl_payload *py;
  char *actual;

  if(vx == NULL){
    return NULL;
  }

  py = vx->v_payload;
  if(py == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "logic failure: variable has a null payload\n");
    abort();
#endif
    return NULL;
  }

  type = py->p_type;
  if(type >= KATCP_MAX_VRT){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "logic failure: variable has payload has invalid type\n");
    abort();
#endif
    return NULL;
  }

  actual = path_suffix_vrbl_katcp(d, path);

#ifdef DEBUG
  fprintf(stderr, "find payload: full path is %s suffix is %s\n", path, actual);
#endif

  return (*(ops_type_vrbl[type].t_element))(d, vx, py, actual);
}

int type_payload_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  struct katcp_vrbl_payload *ty;
  
  if(py == NULL){
    if(vx == NULL){
      return -1;
    }
    ty = vx->v_payload;
  } else {
    ty = py;
  }

  if(ty->p_type >= KATCP_MAX_VRT){
    return -1;
  }

  return ty->p_type;
}

int find_type_vrbl_katcp(struct katcp_dispatch *d, struct katcp_vrbl *vx, char *path)
{
  struct katcp_vrbl_payload *py;

  py = find_payload_katcp(d, vx, path);

  if(py == NULL){
    return -1;
  }

  return type_payload_vrbl_katcp(d, vx, py);
}

struct katcp_vrbl *create_vrbl_katcp(struct katcp_dispatch *d, unsigned int type, unsigned int flags, void *state, int (*refresh)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), int (*change)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx), void (*release)(struct katcp_dispatch *d, void *state, char *name, struct katcp_vrbl *vx))
{
  /* WARNING: should probably be deprecated - scan_vrbl does all of this and more */
  struct katcp_vrbl *vx;

  vx = allocate_vrbl_katcp(d, type);
  if(vx == NULL){
    return NULL;
  }

  if(configure_vrbl_katcp(d, vx, flags, state, refresh, change, release)){
    /* WARNING: gets called with null name, but fortunately no release callback yet installed */
    destroy_vrbl_katcp(d, NULL, vx);
    return NULL;
  }

  return vx;
}

void destroy_vrbl_katcp(struct katcp_dispatch *d, char *name, struct katcp_vrbl *vx)
{
  /* WARNING: risky api - vx might still be in tree ... */
  if(vx == NULL){
    return;
  }

  vx->v_change = NULL;
  vx->v_refresh = NULL;

  if(vx->v_release){
    (*(vx->v_release))(d, vx->v_extra, name ? name : vx->v_name, vx);
    vx->v_release = NULL;
  }

  vx->v_extra = NULL;

  release_vrbl_katcp(d, vx, vx->v_payload);

  vx->v_status = 0;
  vx->v_flags = 0;

  vx->v_name = NULL;

  free(vx);
}

/************************************************************************************/

int add_payload_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx, struct katcp_vrbl_payload *py)
{
  struct katcp_vrbl_payload *ty;

  if(vx == NULL){
    return -1;
  }

#ifdef KATCP_CONSISTENCY_CHECKS
  if(vx->v_flags & KATCP_VRF_HID){
    fprintf(stderr, "logic failure: attempting to display hidden variable %p\n", vx);
    abort();
  }
#endif

  if(py == NULL){
    ty = vx->v_payload;
    if(ty == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
      fprintf(stderr, "logic failure: no variable type to output for %p\n", vx);
      abort();
#endif
      return -1;
    }
  } else {
    ty = py;
  }
  
  if(ty->p_type >= KATCP_MAX_VRT){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "logic failure: invalid type %u of payload in variable %p\n", ty->p_type, vx);
    abort();
#else
    return -1;
#endif
  }

  return (*(ops_type_vrbl[ty->p_type].t_add))(d, px, flags, vx, ty);
}

#if 0
int add_vrbl_katcp(struct katcp_dispatch *d, struct katcl_parse *px, int flags, struct katcp_vrbl *vx)
{
  if(vx == NULL){
    return -1;
  }

  return add_payload_katcp(d, px, flags, vx, vx->v_payload);
}
#endif

/* variable region functions ******************************************************/

static struct katcp_vrbl *find_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, char *key)
{
  if(rx == NULL){
    return NULL;
  }

  return find_data_avltree(rx->r_tree, key);
}

static int insert_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, struct katcp_vrbl *vx, char *key)
{
  struct avl_node *node;

  if(rx == NULL){
    return -1;
  }

  node = store_exposed_node_avltree(rx->r_tree, key, vx);
  if(node == NULL){
    return -1;
  }

  /* WARNING v_name will become invalid whenever this is removed from tree */
  /* TODO - check that variable deletion unsets this ? */
  vx->v_name = node->n_key;

  return 0;
}

static int remove_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, struct katcp_vrbl *vx)
{
  struct avl_node *n;

  if(vx->v_name == NULL){
#ifdef DEBUG
    fprintf(stderr, "remove region: possible problem - variable %p has no name, thus difficult to remove\n", vx);
#endif
    return -1;
  }

  n = find_name_node_avltree(rx->r_tree, vx->v_name);
  if(n == NULL){
    return -1;
  }

  /* v_name shared by node and vx, need to be deallocated concurrently */
  
  destroy_vrbl_katcp(d, vx->v_name, vx);

  if(del_node_avltree(rx->r_tree, n, NULL) < 0){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "major logic problem: avl delete should not fail after node has been found\n");
    abort();
#endif
    return -1;
  }

  return 0;
}

/* full variable layer functions **************************************************/

struct katcp_vrbl *find_vrbl_katcp(struct katcp_dispatch *d, char *key)
{
  struct katcp_flat *fx;

  fx = this_flat_katcp(d);

  return update_vrbl_katcp(d, fx, key, NULL, 0);
}

#if 0
/* YAY: not needed, can use update */
struct katcp_vrbl *find_all_vrbl_katcp(struct katcp_dispatch *d, char *key)
{
  struct katcp_region *vra[MAX_DEPTH_VRBL];
  struct katcp_flat *fx;
  struct katcp_group *gx;
  struct katcp_shared *s;
  struct katcp_vrbl *vx;
  unsigned int i;

  fx = this_flat_katcp(d);
  gx = this_group_katcp(d);
  s = d->d_shared;

  if((fx == NULL) || (gx == NULL) || (s == NULL)){
    return NULL;
  }

  vra[0] = fx->f_region;
  vra[1] = gx->g_region;
  vra[2] = s->s_region;

  for(i = 0; i < MAX_DEPTH_VRBL; i++){
    vx = find_region_katcp(d, vra[i], key);
    if(vx){
#ifdef DEBUG
      fprintf(stderr, "find variable: located %s (%p) at depth %u\n", key, vx, i);
#endif
      return vx;
    }
  }

  return NULL;
}
#endif

/* traverse the hierachy of variables regions ****************/

int traverse_vrbl_katcp(struct katcp_dispatch *d, void *state, int (*callback)(struct katcp_dispatch *d, void *state, char *key, void *data))
{
  struct katcp_region *vra[MAX_DEPTH_VRBL];
  struct katcp_flat *fx;
  struct katcp_group *gx;
  struct katcp_shared *s;
  unsigned int i;

  fx = this_flat_katcp(d);
  gx = this_group_katcp(d);
  s = d->d_shared;

  if((fx == NULL) || (gx == NULL) || (s == NULL)){
#ifdef DEBUG
    fprintf(stderr, "traverse: insufficient context available\n");
#endif
    return -1;
  }

  vra[0] = fx->f_region;
  vra[1] = gx->g_region;
  vra[2] = s->s_region;

  for(i = 0; i < MAX_DEPTH_VRBL; i++){
#ifdef DEBUG
    fprintf(stderr, "traverse: checking[%d]=%p\n", i, vra[i]);
#endif
    complex_inorder_traverse_avltree(d, vra[i]->r_tree->t_root, state, callback);
  }

  return 0;
}

/* 
 * exact absolute
 * C> var@root*                    (1)
 * E> group*var@group*             (2)
 * G> group*flat*var@flat*         (3)
 *
 * exact relative
 * D> *var@curgrouponly*           (2)
 * F> **var@curflatonly*           (3)
 *
 * search
 * A> varinflatthengroupthenroot   (0)  
 * B> *varinflatthengroup          (1)
 */

static int tokenise_vrbl_katcp(struct katcp_dispatch *d, char *str, unsigned int *vector)
{
  /* WARNING: this function needs a vector of MAX_COUNT_STAR + 1 */
  int count, i;

  count = 0;
  i = 0;

  while(str[i] != '\0'){
    switch(str[i]){
      case KATCP_VRBL_DELIM_GROUP :
        if(count >= MAX_COUNT_STAR){
          return -1;
        }
        vector[count] = i;
        str[i] = '\0';
        count++;
        i++;
        break;
      case KATCP_VRBL_DELIM_TREE    :
      case KATCP_VRBL_DELIM_ARRAY   :
#if 0
      case KATCP_VRBL_DELIM_LOGIC :
#endif
        vector[count] = i;
        str[i] = '\0';
        return count;
      default :
        i++;
        break;
    }
  }

  vector[count] = i;

  if(i <= 0){
    return -1;
  }

  return count;
}

int for_all_flats_vrbl_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, char *name, void *state, int (*callback)(struct katcp_dispatch *d, void *state, struct katcp_flat *fx))
{
  /* WARNING: naive: does not check if the variable exists AND pessimistic: assumes the worst for search paths */
  struct katcp_flat *ft;
  struct katcp_group *gt;
  struct katcp_shared *s;
  int count, last, result, i, j;
  unsigned int v[MAX_COUNT_STAR + 1];
  char *copy;

  s = d->d_shared;

  copy = strdup(name);
  if(copy == NULL){
    return -1;
  }

  count = tokenise_vrbl_katcp(d, copy, v);
  if(count < 0){
    free(copy);
    return -1;
  }

  if(v[count] <= 0){
    return -1;
  }

  last = v[count] - 1;

#ifdef KATCP_CONSISTENCY_CHECKS
  if(fx){
    if(fx->f_group == NULL){
      fprintf(stderr, "major logic problem: duplex instance %p (%s) not a member of any group\n", fx, fx->f_name);
      abort();
    }
  }
  if(s == NULL){
    fprintf(stderr, "major logic problem: no global state available\n");
    abort();
  }
#endif

  ft = NULL;
  gt = NULL;

  switch(count){
    case 0 : /* A> search in current flat, then current group, then root */
      /* assume the worst ... make it the entire space ... */
      break;
    case 1 :
      if(v[0] == 0){ /* B> search in current flat, then current group */
        if(fx == NULL){
          free(copy);
          return -1;
        }
        gt = fx->f_group;
      } else if(v[0] == last){ /* C> search in root */
        /* entire space */
      } else {
        free(copy);
        return -1;
      }
      break;
    case 2 : 
      if(v[1] != last){
        free(copy);
        return -1;
      }
      if(v[0] == 0){ /* D> search in current group only */
        if(fx == NULL){
          free(copy);
          return -1;
        }
        gt = fx->f_group;
      } else { /* E> search in named group */
        gt = find_group_katcp(d, copy);
        if(gt == NULL){
#ifdef DEBUG
          fprintf(stderr, "variable update: no group %s found while searching for %s\n", copy, name);
#endif
          free(copy);
          return -1;
        }
      }
      break;
    case 3 : 
      if(v[0] == 0){ /* F> search in current flat */
        if(v[1] != 1){
          free(copy);
          return -1;
        }
        ft = fx;
      } else { /* G> search in named flat */

        ft = find_name_flat_katcp(d, copy + 1, copy + v[0] + 1, 1);
        if(ft == NULL){
          free(copy);
          return -1;
        }
      }
      break;
    default :
      free(copy);
      return -1;
  }

  /* TODO: shouldn't we show the variable to the callback function ? */
  free(copy);

  if(ft){
    return (*(callback))(d, state, ft);
  }

  result = 0;

  if(gt){
    for(i = 0; i < gt->g_count; i++){
      ft = gt->g_flats[i];
      if((*(callback))(d, state, ft) < 0){
        result = (-1);
      }
    }
    return result;
  }

  for(j = 0; j < s->s_members; j++){
    gt = s->s_groups[j];
    if(gt){
      for(i = 0; i < gt->g_count; i++){
        ft = gt->g_flats[i];
        if((*(callback))(d, state, ft) < 0){
          result = (-1);
        }
      }
    }
  }

  return result;
}

struct katcp_vrbl *update_vrbl_katcp(struct katcp_dispatch *d, struct katcp_flat *fx, char *name, struct katcp_vrbl *vo, int clobber)
{
  struct katcp_shared *s;
  struct katcp_group *gx;
  struct katcp_flat *fy;
  struct katcp_vrbl *vx;
  struct katcp_region *vra[MAX_DEPTH_VRBL], *rx;
  int i, count, last;
  char *copy, *var;
  unsigned int v[MAX_COUNT_STAR + 1];

  s = d->d_shared;

  copy = strdup(name);
  if(copy == NULL){
    return NULL;
  }

  count = tokenise_vrbl_katcp(d, copy, v);
  if(count < 0){
    free(copy);
    return NULL;
  }

  if(v[count] <= 0){
    return NULL;
  }

  last = v[count] - 1;

#ifdef KATCP_CONSISTENCY_CHECKS
  if(fx){
    if(fx->f_group == NULL){
      fprintf(stderr, "major logic problem: duplex instance %p (%s) not a member of any group\n", fx, fx->f_name);
      abort();
    }
  }
  if(s == NULL){
    fprintf(stderr, "major logic problem: no global state available\n");
    abort();
  }
#endif

  vra[0] = NULL;
  vra[1] = NULL;
  vra[2] = NULL;

  var = NULL;

#ifdef DEBUG
  fprintf(stderr, "variable update: in %s doing a %d star search, stars at", name, count);
  for(i = 0; i < count; i++){
    fprintf(stderr, " %d", v[i]);
  }
  fprintf(stderr, " and last position %d\n", last);
#endif

  switch(count){
    case 0 : /* A> search in current flat, then current group, then root */
      if(fx == NULL){
        free(copy);
        return NULL;
      }
      var = copy;
      vra[0] = fx->f_region;
      vra[1] = fx->f_group->g_region;
      vra[2] = s->s_region;
      break;
    case 1 :
      if(v[0] == 0){ /* B> search in current flat, then current group */
        if(fx == NULL){
          free(copy);
          return NULL;
        }
        var = copy + 1;
        vra[0] = fx->f_region;
        vra[1] = fx->f_group->g_region;
      } else if(v[0] == last){ /* C> search in root */
        var = copy;
        vra[0] = s->s_region;
      } else {
        free(copy);
        return NULL;
      }
      break;
    case 2 : 
      if(v[1] != last){
        free(copy);
        return NULL;
      }
      if(v[0] == 0){ /* D> search in current group only */
        if(fx == NULL){
          free(copy);
          return NULL;
        }
        var = copy + 1;
        vra[0] = fx->f_group->g_region;
      } else { /* E> search in named group */
        gx = find_group_katcp(d, copy);
        if(gx == NULL){
#ifdef DEBUG
          fprintf(stderr, "variable update: no group %s found while searching for %s\n", copy, name);
#endif
          free(copy);
          return NULL;
        }
        vra[0] = gx->g_region;
        var = copy + v[0] + 1;
      }
      break;
    case 3 : 
      if(v[0] == 0){ /* F> search in current flat */
        if(v[1] != 1){
          free(copy);
          return NULL;
        }
        vra[0] = fx->f_region;
        var = copy + 2;
      } else { /* G> search in named flat */

        fy = find_name_flat_katcp(d, copy + 1, copy + v[0] + 1, 1);
        if(fy == NULL){
          free(copy);
          return NULL;
        }

        vra[0] = fy->f_region;
        var = copy + v[1] + 1;
      }
      break;
    default :
      free(copy);
      return NULL;
  }

  if((var == NULL) || (var[0] == '\0')){
#ifdef DEBUG
    fprintf(stderr, "variable update: no valid variable name decoded\n");
#endif
    free(copy);
    return NULL;
  }

#ifdef KATCP_CONSISTENCY_CHECKS
  if(vra[0] == NULL){
    fprintf(stderr, "major logic problem: attempting to update variable without any target location\n");
    abort();
  }
#endif

  vx = NULL;
  i = 0;
  rx = NULL;

  for(i = 0; i < MAX_DEPTH_VRBL; i++){
    if(vra[i]){
      vx = find_region_katcp(d, vra[i], var);
#ifdef DEBUG
      fprintf(stderr, "variable update: search of region [%d]=%p for %s yields %p\n", i, vra[i], var, vx);
#endif
      if(vx){
        rx = vra[i];
        i = MAX_DEPTH_VRBL;
      }
    } else {
      i = MAX_DEPTH_VRBL;
    }
  } 

  if(vx == NULL){
    if(vo == NULL){
      free(copy);
      return NULL;
    }

    if(insert_region_katcp(d, vra[0], vo, var) < 0){
      free(copy);
      return NULL;
    }

    free(copy);
    return vo;

  } else {

    if(vo == NULL){ /* no replacement given, so different meaning: delete or find */
      if(clobber){ /* delete entry - replace it with something null */
        free(copy);
        if(remove_region_katcp(d, rx, vx) < 0){
          /* WARNING: sadly this is unlikely to be triggered, more likely that there will be a not found case, which then returns NULL */
          return vx;
        } else {
          return NULL;
        }
      } else {  /* just find */
        free(copy);
        return vx;
      }
    }

    if(vo){
      /* TODO: replace content of variable */
#if 1
      fprintf(stderr, "usage problem: no insitu update implemented yet\n");
      abort();
#endif
    }

    free(copy);
    return vx;
  }

}

/* setup logic ***********************************************/

void destroy_vrbl_for_region_katcp(void *global, char *key, void *payload)
{
  struct katcp_dispatch *d;
  struct katcp_vrbl *vx;

  d = global;
  vx = payload;

  destroy_vrbl_katcp(d, key, vx);
}

void destroy_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx)
{
  if(rx == NULL){
    return;
  }

  if(rx->r_tree){
    destroy_complex_avltree(rx->r_tree, d, &destroy_vrbl_for_region_katcp);
    rx->r_tree = NULL;
  }

  free(rx);
}

struct katcp_region *create_region_katcp(struct katcp_dispatch *d)
{
  struct katcp_region *rx;

  rx = malloc(sizeof(struct katcp_region));
  if(rx == NULL){
    return NULL;
  }

  rx->r_tree = create_avltree();

  if(rx->r_tree == NULL){
    destroy_region_katcp(d, rx);
    return NULL;
  }

  return rx;
}

/* top-level string API functions *********************************************/

struct katcp_vrbl *create_string_vrbl_katcp(struct katcp_dispatch *d, unsigned int options, char *value)
{
  struct katcp_vrbl *vx;

  vx = create_vrbl_katcp(d, KATCP_VRT_STRING, options, NULL, NULL, NULL, NULL);
  if(vx == NULL){
    return NULL;
  }

  if(set_string_vrbl_katcp(d, vx, NULL, value) < 0){
    destroy_vrbl_katcp(d, NULL, vx);
    return NULL;
  }

  return vx;
}

int make_string_vrbl_katcp(struct katcp_dispatch *d, struct katcp_group *gx, char *key, unsigned int options, char *value)
{
  struct katcp_region *rx;
  struct katcp_shared *s;
  struct katcp_vrbl *vx;

  s = d->d_shared;

  if(gx){
    rx = gx->g_region;
  } else {
    rx = s->s_region;
  }

  if(rx == NULL){
    return -1;
  }

  if(find_region_katcp(d, rx, key)){
    return -1;
  }

  vx = create_string_vrbl_katcp(d, options, value);
  if(vx == NULL){
    return -1;
  }

  if(insert_region_katcp(d, rx, vx, key) < 0){
    destroy_vrbl_katcp(d, key, vx);
    return -1;
  }

  return 0;
} 

/****/

#if 0

int insert_string_region_katcp(struct katcp_dispatch *d, struct katcp_region *rx, char *key, unsigned int flags, char *value)
{
  struct katcp_vrbl *vx;

  vx = create_string_vrbl_katcp(d, flags, value);
  if(vx == NULL){
    return -1;
  }

  if(insert_region_katcp(d, rx, vx, key) < 0){
    destroy_vrbl_katcp(d, key, vx);
    return -1;
  }

  return 0;
}
#endif

/* commands operating on variables ********************************************/

int var_declare_group_cmd_katcp(struct katcp_dispatch *d, int argc)
{
  char *name, *copy, *ptr, *current, *path;
  int fresh;
  unsigned int result, type, options, flag;
  struct katcp_vrbl *vx;
  struct katcp_flat *fx;

  if(argc < 2){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "need a name");
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  name = arg_string_katcp(d, 1);
  if(name == NULL){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unable to process a null name");
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  options = 0;
  type = KATCP_VRT_GONE;

  if(argc > 2){
    copy = arg_copy_string_katcp(d, 2);
    if(copy == NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "type field may not be null");
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }

    current = copy;

    do{
      ptr = strchr(current, ',');
      if(ptr){
        ptr[0] = '\0';
      }

      flag = flag_from_string_vrbl_katcp(d, current);
      if(flag){
        options |= flag;
      } else {
        result = type_from_string_vrbl_katcp(d, current);

        if(result >= KATCP_MAX_VRT){
          log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unknown variable attribute %s for %s", current, name);
          free(copy);
          return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
        }

        if((type < KATCP_MAX_VRT) && (type != result)){
          log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "conflicting type attribute %s for %s", current, name);
          free(copy);
          return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
        }

        type = result;
      }

      if(ptr){
        current = ptr + 1;
      }

    } while(ptr != NULL);

    free(copy);
  }

  if(type >= KATCP_MAX_VRT){
    type = KATCP_VRT_STRING;
  }

  path = path_suffix_vrbl_katcp(d, name);

  if(argc > 3){
    if(path != NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unwilling to declare variable %s with external path", path);
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }
    path = arg_string_katcp(d, 3);
    if(path == NULL){
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }
  }

  fx = this_flat_katcp(d);
  if(fx == NULL){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_API);
  }

  vx = find_vrbl_katcp(d, name);

  if(vx){
    if(path == NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unwilling to redeclare variable %s", name);
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }

    if((options != 0) && (options != vx->v_flags)){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unwilling to change attributes of %s", name);
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }

    fresh = 0;
  } else {
    fresh = 1;
  }

  vx = scan_vrbl_katcp(d, vx, NULL, path, 1, type);
  if(vx == NULL){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unable to initialise variable %s with null scan", name);
    return KATCP_RESULT_FAIL;
  }

  if(fresh){
    if(configure_vrbl_katcp(d, vx, options, NULL, NULL, NULL, NULL) < 0){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unable to configure new variable %s", name);
      /* can only destroy a fresh variable, the others already in tree */
      destroy_vrbl_katcp(d, name, vx);
      return KATCP_RESULT_FAIL;
    }

    if(update_vrbl_katcp(d, fx, name, vx, 0) == NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unable to declare new variable %s", name);
      /* can only destroy a fresh variable, the others already in tree */
      destroy_vrbl_katcp(d, name, vx);
      return KATCP_RESULT_FAIL;
    }
  }

  log_message_katcp(d, KATCP_LEVEL_DEBUG, NULL, "%s variable %s with type %u and flags 0x%x", fresh ? "created" : "updated", name, type, options);

  return KATCP_RESULT_OK;
}

/* variable listing - callback then command ***************************/

int var_list_callback_katcp(struct katcp_dispatch *d, void *state, char *key, struct katcp_vrbl *vx)
{
#define BUFFER 128
  char *ptr, *tmp;
  unsigned int i;
  unsigned int used, have, len;
  struct katcp_vrbl_payload *py;

#ifdef KATCP_CONSISTENCY_CHECKS
  if((vx == NULL) || (vx->v_payload == NULL)){
    fprintf(stderr, "variables: major logic problems: null data structure passed to listing function\n");
    abort();
  }
#endif

#ifdef DEBUG
  fprintf(stderr, "variable: about to list %s at %p\n", key, vx);
#endif

  if(vx->v_flags & KATCP_VRF_HID){
    return 0;
  }

  py = find_payload_katcp(d, vx, key);
  if(py == NULL){
#ifdef DEBUG
    fprintf(stderr, "variable: no matching payload field found for variable %s\n", key);
#endif
    return -1;
  }

  ptr = malloc(BUFFER);
  if(ptr == NULL){
    return -1;
  }
  have = BUFFER;

  tmp = type_to_string_vrbl_katcp(d, py->p_type);
  if(tmp == NULL){
    free(ptr);
    return -1;
  }

  len = strlen(tmp);

  memcpy(ptr, tmp, len + 1);
  used = len;

  for(i = 0; flag_lookup_vrbl[i]; i++){
    if((1 << i) & vx->v_flags){
#ifdef DEBUG
      fprintf(stderr, "vrbl: flag match[%u]=%s\n", i, flag_lookup_vrbl[i]);
#endif
      len = strlen(flag_lookup_vrbl[i]);
      if((used + 1 + len + 1) > have){
        have = used + 1 + len + 1;
        tmp = realloc(ptr, have);
        if(tmp == NULL){
          free(ptr);
          return -1;
        }
        ptr = tmp;
      }
      ptr[used] = ',';
      used++;
      memcpy(ptr + used, flag_lookup_vrbl[i], len + 1);
      used += len;
    }
  }

  prepend_inform_katcp(d);
  append_string_katcp(d, KATCP_FLAG_STRING, key);
  append_string_katcp(d, KATCP_FLAG_STRING, ptr);
  append_payload_vrbl_katcp(d, KATCP_FLAG_PRETTY | KATCP_FLAG_LAST, vx, py);

  free(ptr);

  return 0;
#undef BUFFER
}

int var_list_void_callback_katcp(struct katcp_dispatch *d, void *state, char *key, void *data)
{
  struct katcp_vrbl *vx;

  vx = data;

  return var_list_callback_katcp(d, state, key, vx);
}

int var_list_group_cmd_katcp(struct katcp_dispatch *d, int argc)
{
  char *key;
  unsigned int i;
  int result;
  struct katcp_vrbl *vx;

#if 0
  struct katcp_flat *fx;
  struct katcp_group *gx;
  struct katcp_shared *s;

  fx = this_flat_katcp(d);
  gx = this_group_katcp(d);
  s = d->d_shared;

  if((fx == NULL) || (gx == NULL) || (s == NULL)){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_API);
  }
#endif

  result = 0;

  if(argc > 1){
    for(i = 1 ; i < argc ; i++){
      key = arg_string_katcp(d, i);
      if(key == NULL){
        return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
      }
      vx = find_vrbl_katcp(d, key);
      if(vx == NULL){
        log_message_katcp(d, KATCP_LEVEL_WARN, NULL, "%s not found", key);
        result = (-1);
      } else {
        if(var_list_callback_katcp(d, NULL, key, vx) < 0){
          result = (-1);
        }
      }
    }
  } else {
    result = traverse_vrbl_katcp(d, NULL, &var_list_void_callback_katcp);
  }

  if(result < 0){
    return KATCP_RESULT_FAIL;
  }

  return KATCP_RESULT_OK;
}

/* set a previously declared variable ********************************/

int var_set_group_cmd_katcp(struct katcp_dispatch *d, int argc)
{
  char *key, *value, *path, *ptr;
  struct katcp_vrbl *vx;
  unsigned int type;

  if(argc <= 2){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  key = arg_string_katcp(d, 1);
  if(key == NULL){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  vx = find_vrbl_katcp(d, key);
  if(vx == NULL){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "variable %s not found", key);
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_NOT_FOUND);
  }

  value = arg_string_katcp(d, 2);
  if(value == NULL){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  if(argc > 3){
    ptr = arg_string_katcp(d, 3);
    if(ptr == NULL){
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }
    type = type_from_string_vrbl_katcp(d, ptr);

    if(type >= KATCP_MAX_VRT){
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }

  } else {
    type = KATCP_VRT_STRING;
  }

  path = path_suffix_vrbl_katcp(d, key);

  if(argc > 4){
    if(path != NULL){
      log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unwilling to set variable %s with external path", path);
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }
    path = arg_string_katcp(d, 4);
    if(path == NULL){
      return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
    }
  }

  if(scan_vrbl_katcp(d, vx, value, path, 1, type) < 0){
    return KATCP_RESULT_FAIL;
  }

  return KATCP_RESULT_OK;
}

int var_delete_group_cmd_katcp(struct katcp_dispatch *d, int argc)
{
  char *key;
  struct katcp_vrbl *vx;
  struct katcp_flat *fx;

  fx = this_flat_katcp(d);
  if(fx == NULL){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_API);
  }

  if(argc < 2){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  key = arg_string_katcp(d, 1);
  if(key == NULL){
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_USAGE);
  }

  log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "attempting to locate %s for deletion", key);

  vx = find_vrbl_katcp(d, key);
  if(vx == NULL){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "variable %s not found", key);
    return extra_response_katcp(d, KATCP_RESULT_FAIL, KATCP_FAIL_NOT_FOUND);
  }

  if(update_vrbl_katcp(d, fx, key, NULL, 1)){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL, "unable to remove variable %s", key);
    return KATCP_RESULT_FAIL;
  }

  return KATCP_RESULT_OK;
}

#endif
