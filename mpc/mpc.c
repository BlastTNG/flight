/* desecrate the C preprocessor to extract this file's SVN revision */

/* Fun fact: GCC's CPP allows $ in identifier names */
#define $Rev (0?0 /* eat a : here */
#define $    )

const static inline int ProtoRev(void) { return $Rev: 6019 $; };

#undef $Rev
#undef $
/* end preprocessor desecration */

int main(void)
{
  return 0;
}
