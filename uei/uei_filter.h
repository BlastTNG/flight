/* 
 * uei_filter.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 * Created on: Aug 11, 2014 by seth
 *
 * This is a 3rd order Butterworth filter with corner frequency (-3dB) at
 * 100Hz.  It is specifically designed for the gyroscope output at 200Hz.
 *
 * If you are looking for the analog channels (lower frequency output), you
 * want uei_fir.h.
 */

#ifndef UEI_FILTER_H_
#define UEI_FILTER_H_


#define FILTORDER 3
#define GAIN 5.525187588e+01

typedef struct {
    int i_val;
    float x_val[FILTORDER+1];
    float y_val[FILTORDER+1];
} uei_filter_t;


static const float yb[FILTORDER] = { 0.2780599176, -1.1828932620, 1.7600418803};


static inline float uei_filter_get (uei_filter_t *m_filter)
{
    return m_filter->y_val[m_filter->i_val];
}

static inline void uei_filter_put(uei_filter_t *m_filter, float m_val)
{
    int i3 = (m_filter->i_val + 1) & 3;
    int i0 = (m_filter->i_val + 2) & 3;
    int i1 = (m_filter->i_val + 3) & 3;
    int i2 = m_filter->i_val;

    m_filter->x_val[i3] = m_val / GAIN;
    m_filter->y_val[i3] = (m_filter->x_val[i0] + m_filter->x_val[i3]) +
            3 * (m_filter->x_val[i1] + m_filter->x_val[i2]) +
            yb[0] * m_filter->y_val[i0] + yb[1] * m_filter->y_val[i1] + yb[2] * m_filter->y_val[i2];
    m_filter->i_val = i3;

}
#endif /* UEI_FILTER_H_ */
