/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/


int setupmpc(struct mpc_channel_struct *);

void coaddslow (int, struct mpc_channel_struct *, unsigned *);

void coaddfast (int, struct mpc_channel_struct *, unsigned *);

