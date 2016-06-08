/**
 * @file slinger_time.h
 *
 * @date Sept 5, 2012
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2012 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef SLINGER_TIME_H_
#define SLINGER_TIME_H_

#include <inttypes.h>
#include <stdint.h>


#define EBEX_TIME_MAX_TICKS (131836U << 14)
#define EBEX_TIME_TOTAL_MAX (INT64_C(131836) << 28)

/**
 * EBEX_TIME_MASK takes the lower 46 bits of the timeword (ticks + period) to allow
 * for time-only comparison.  Higher bits are for board ID and validity
 */
#define EBEX_TIME_MASK(x) ((uint64_t)(x) & UINT64_C(0x3fffffffffff))
typedef union
{
	uint64_t		qword;
	struct
	{
		union
		{
			struct
			{
				uint32_t		intlow:14;
				uint32_t		intmid:18;
			} __attribute__((packed)) acs;
			struct
			{
				uint32_t	ticks;
			} bolo;
		};

		uint32_t	period:14;
		uint32_t	board_id:2;
		uint32_t	time_ok:4;
		uint16_t	unused:12;
	} __attribute__((packed));
} slinger_time_t;

extern int64_t slinger_tick_diff;

static inline int64_t slinger_time_diff(const slinger_time_t *m_x, const slinger_time_t *m_y)
{
	int64_t x, y;
	int64_t period_step = EBEX_TIME_MAX_TICKS;

	x = m_x->period * period_step + m_x->bolo.ticks;
	y = m_y->period * period_step + m_y->bolo.ticks;

	return x-y;
}

static inline void slinger_time_add(const slinger_time_t *m_src, int32_t m_ticks, slinger_time_t *m_dest)
{
	m_dest->qword = m_src->qword;

	m_dest->bolo.ticks += m_ticks;
	if (m_dest->bolo.ticks >= EBEX_TIME_MAX_TICKS)
	{
		m_dest->period++;
		m_dest->bolo.ticks -= EBEX_TIME_MAX_TICKS;
	}

}

#define slinger_time_sub(_src, _ticks, _dest) slinger_time_add(_src, -(_ticks), _dest)

/**
 * Ensures that any timeword passed to it will be represented as the time for a single time server
 * @param m_src Source timeword
 * @param m_dest Timeword that will be in ACS timestamp time
 */
static inline void slinger_time_normalize(const slinger_time_t *m_src, slinger_time_t *m_dest)
{
	int64_t board0;
	int64_t board1;

	m_dest->qword = 0;

	if (m_src->board_id == 0)
	{
		board0 = ((int64_t)m_src->period) * EBEX_TIME_MAX_TICKS + m_src->bolo.ticks;
		board1 = board0 + slinger_tick_diff;
		while (board1 < 0) board1 += EBEX_TIME_TOTAL_MAX;
		while (board1 >= EBEX_TIME_TOTAL_MAX) board1 -= EBEX_TIME_TOTAL_MAX;

		m_dest->board_id = 1;

		m_dest->period = (uint32_t) (board1 / EBEX_TIME_MAX_TICKS);
		m_dest->bolo.ticks = board1 % EBEX_TIME_MAX_TICKS;
	}
	else
		m_dest->qword = m_src->qword;
}

/**
 * Updates the stored time offset.  Offset is stored such that it should be added to time_bolo.
 *
 * @param m_time_acs Time word from ACS time server (time server 2)
 * @param m_time_bolo Time word from Bolo time server (time server 1)
 */
static inline void slinger_time_update(const slinger_time_t *m_time_acs, const slinger_time_t *m_time_bolo)
{
	if (m_time_acs->time_ok == 5 && m_time_bolo->time_ok == 5)
	{
		slinger_tick_diff = slinger_time_diff(m_time_acs, m_time_bolo);
	}
}


#endif /* SLINGER_TIME_H_ */
