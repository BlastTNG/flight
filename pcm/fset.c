/* pcm: the Spider master control program
 *
 * fset.c: handle routines for field set manipulation
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Field Set notes:
 *
 * A "field set" is a file containing a list of fields (probably mostly
 * bolometers) which will be inserted into the "variable field list" part of
 * the downlink.
 *
 * Up to 255 different field lists may be defined, each is defined in a file
 * called ###.fset located in /data/etc/, numbered 001.fset through 255.fset.
 * Not all 255 field sets need be present. The field set files present are not
 * required to be consequitively numbered or to start with number 000.
 *
 * Not more than FSET_MAXLEN (defined in field_set.h) fields may be defined in
 * any set.  Excess fields are a syntax error.
 *
 * A special field set numbered 0 is only used when *NO* fset files are found
 * by pcm.  This failover comprises zero fields.
 *
 * The fset file definition follows:
 *
 * - Completely blank lines are ignored.  (A line containing only whitespace
 *   isn't blank: it will produce a syntax error.)
 * - Comment lines are allowed.  A comment line consists of a hash ('#') in
 *   the first column of the line followed by anything.  Comment lines are
 *   ignored.
 * - The first non-comment line contains a single decimal integer indicating the
 *   total number of fields in the field set.
 * - Each subsequent (non-comment, non-blank) line defines a field for the set.
 * - The character in column zero of a field line must be one of the following
 *   characters (case sensitive) indicating the type of field:
 *
 *     'n': a non-bolometer field
 *     'b': a bolometer field
 *
 * - The second column of every field line is ignored (typically a space is put
 *   here)
 * - For "normal" fields, the field name starts in column 3 and continues until
 *   the newline.
 * - For bolometer fields, a string of the form "m#c##r##" indicating the MCE
 *   number (0 to 5), MCE column (00 to 15) and MCE row number (00 to 32) of the
 *   desired bolometer.  Column and row are zero padded 2-digit numbers.
 * - The next (non-comment, non-blank) line after the last field defined must
 *   contain 'e' in the first column.
 *
 * A syntax error in a fset file will result in pcm COMPLETELY IGNORING that
 * file (resulting in no fset defined for that file).
 *
 * Syntax errors are:
 * - the first non-comment, non-blank line not consisting solely of a decimal
 *   integer.
 * - the first column of a non-comment, non-blank line containing something
 *   other than n, w, b, or e.
 * - the end of file ('E') line occurring too early or too late, based on the
 *   number of fields declared on line 1, or being missing.
 * - declaring a non-existent field
 * - declaring an out-of-range bolometer
 */

#include "fset.h"
#include "tes.h"
#include "blast.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>

#define FSET_DIR "/data/etc/spider"

/* parse fset file number 'i' and store it in 'set'.  Returns 'set' or NULL
 * on error
 */
struct fset *read_fset(int i, struct fset *set)
{
  struct fset_item *s = NULL;
  int lineno = 0, c = 0, n = -1;
  FILE *stream;
  char line[1024];
  char name[sizeof(FSET_DIR) + sizeof(".fset") + 5];

  sprintf(name, FSET_DIR "/%03i.fset", i);

  /* open */
  if ((stream = fopen(name, "r")) == NULL) {
    /* a missing file isn't interesting */
    if (errno != ENOENT)
      berror(err, "FSet: unable to open %s as FSET%03i", name, i);
    return NULL;
  }

  /* parse lines */
  while (fgets(line, 1024, stream)) {
    size_t len = strlen(line);
    /* check for line length */
    if (line[len - 1] != '\n') {
      bprintf(err, "FSet: Error reading FSET%03i: line %i too long.", i,
          lineno);
      goto LOAD_FSET_ERROR;
    }

    /* skip comments and blank lines */
    if (line[0] == '\n' || line[0] == '#') {
      lineno++;
      continue;
    }

    if (n == -1) {
      /* first line is the number */
      char *endptr;
      n = strtol(line, &endptr, 10);
      /* check for trailing garbage and/or crazy numbers */
      if (*endptr != '\n' || n <= 0 || n >= MAX_FSET) {
        bprintf(err, "FSet: Error reading FSET%03i: bad count on line %i", i,
            lineno);
        goto LOAD_FSET_ERROR;
      }
      s = (struct fset_item *)malloc(sizeof(struct fset_item) * n);
    } else if (c == n) {
      if (line[0] != 'e') {
        bprintf(err, "FSet: Extra field definition on line %i of FSET%03i",
            lineno, i);
        goto LOAD_FSET_ERROR;
      } else {
        /* done -- ignore the rest of the file and return success */
        fclose(stream);
        set->n = n;
        set->f = s;
        return set;
      }
    } else if (line[0] == 'e') { /* EOF */
      bprintf(err, "FSet: Unexpected end on line %i of FSET%03i", lineno, i);
      goto LOAD_FSET_ERROR;
    } else if (line[0] == 'b') { /* bolo */
      /* parse "b m#c##r##\n" */
      if (line[2] != 'm' || line[4] != 'c' || line[7] != 'r' ||
          line[10] != '\n')
      {
        bprintf(err, "FSet: Bad bolometer number on line %i of FSET%03i",
            lineno, i);
        goto LOAD_FSET_ERROR;
      }
      /* lame number parsing.
       * yes ",m" produces the same number as "21" ... DON'T DO THAT */
      s[c].bolo = TESNumber(line[3] - '0',
          (line[5] - '0') * 10 + line[6] - '0',
          (line[8] - '0') * 10 + line[9] - '0');
      if (s[c].bolo < 0) {
        bprintf(err, "FSet: Bad bolometer number on line %i of FSET%03i",
            lineno, i);
        goto LOAD_FSET_ERROR;
      }
      c++;
    } else if (line[0] == 'n') { /* non-bolo */
      s[c].bolo = -1;
      /* check length -- minus two for columns 0 and 1; minus one for \n */
      if (len - 3 > FIELD_LEN) {
        bprintf(err, "FSet: Field name too long on line %i of FSET%03i", lineno,
            i);
        goto LOAD_FSET_ERROR;
      }
      /* TODO: verify field name */
      memcpy(s[c++].name, line + 2, FIELD_LEN);
    } else { /* bad stuff */
      bprintf(err, "FSet: Syntax error on line %i of FSET%03i: %s", lineno, i,
          line);
      goto LOAD_FSET_ERROR;
    }
    lineno++;
  }
  bprintf(err, "FSet: Unexpected EOF reading FSET%03i", i);

LOAD_FSET_ERROR:
  free(s);
  fclose(stream);
  return NULL;
}
