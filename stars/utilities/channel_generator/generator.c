#include <stdio.h>
#include <string.h>
#include "xsc_protocol.h"

typedef enum
{
    ctr_fcp,
    tr_m_azvel,
    tr_r_azvel,
    tr_m_totvel,
    tr_m_azacc,
    point_az,
    point_el,
    point_sigma,
    point_az_trim,
    point_el_trim,
    num_fcp_channels
}
FCPChannelNames;

void fill_slots(XSCChannelInfo* infos, unsigned int num_channels)
{
    unsigned int current_relative_loop = 0;
    unsigned int current_slot = 0;
    unsigned int size = 0;
    for (unsigned int i=0; i<num_channels; i++) {
        size = 1;
        if (infos[i].form == xN_wide_slow || infos[i].form == xN_wide_fast) {
            size = 2;
        }
        if (current_slot + size - 1 < 64) {
            // put it in this loop
            infos[i].relative_loop_num = current_relative_loop;
            infos[i].slot_num = current_slot;
            current_slot += size;
        } else {
            // put it in the next loop
            current_relative_loop++;
            current_slot = 0;
            infos[i].relative_loop_num = current_relative_loop;
            infos[i].slot_num = current_slot;
            current_slot += size;
        }
    }
    if (current_relative_loop >= 2) {
        printf("WARNING: using more than 2 loops per star camera");
    }
}

void init_fcp_channels(XSCChannelInfo* ci)
{
    double identity[2] = {1.0, 0.0};
    xsc_init_channel_info(&ci[ctr_fcp], "ctr_fcp", xN_wide_fast, identity);
    xsc_init_channel_info(&ci[tr_m_azvel], "tr_m_azvel", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[tr_r_azvel], "tr_r_azvel", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[tr_m_totvel], "tr_m_totvel", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[tr_m_azacc], "tr_m_azacc", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[point_az], "point_az", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[point_el], "point_el", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[point_sigma], "point_sigma", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[point_az_trim], "point_az_trim", xN_narrow_slow, identity);
    xsc_init_channel_info(&ci[point_el_trim], "point_el_trim", xN_narrow_slow, identity);
}

void init_infos(XSCChannelInfo* infos)
{
    XSCChannelInfo ci_from_fcp[num_fcp_channels];
    init_fcp_channels(ci_from_fcp);

    XSCServerData sd;
    XSCChannelInfos ci_struct_from_stars;
    xsc_clear_server_data(&sd);
    xsc_init_server_data_and_infos(&sd, &ci_struct_from_stars);

    for (unsigned int i=0; i<num_fcp_channels; i++) {
        infos[i] = ci_from_fcp[i];
    }
    for (unsigned int i=0; i<xN_num_channels; i++) {
        infos[i + num_fcp_channels] = ci_struct_from_stars.infos[i];
    }
}

int main()
{
    unsigned int num_channels = xN_num_channels + num_fcp_channels;
    XSCChannelInfo infos[num_channels];

    init_infos(infos);
    fill_slots(infos, num_channels);

    unsigned int starting_loops[2] = {6, 10};
    for (unsigned int form=0; form<4; form++) {
        if (form == xN_narrow_slow) {
            printf("Narrow Slow channels:\n");
        } else if (form == xN_narrow_fast) {
            printf("Narrow Fast channels:\n");
        } else if (form == xN_wide_slow) {
            printf("Wide Slow channels:\n");
        } else if (form == xN_wide_fast) {
            printf("Wide Fast channels:\n");
        }

        char name_string[31+2+1] = ""; // max name length + two quotes + null terminator
        char loop_string[7] = "";
        for (unsigned int which=0; which<2; which++) {
            for (unsigned int i=0; i<num_channels; i++) {
                if (form == infos[i].form) {
                    sprintf(name_string, "\"x%i_%s\"", which, infos[i].tx_name);

                    // one special case
                    if ((which == 1) && (strcmp(infos[i].tx_name, "ctr_fcp") == 0)) {
                        sprintf(name_string, "\"x0_lt_age_cs\"");
                    }

                    sprintf(loop_string, "LOOP%i", starting_loops[which] + infos[i].relative_loop_num);
                    printf("  {%-33s, 'w', %6s, %2i,},\n",
                        name_string,
                        loop_string,
                        infos[i].slot_num
                    );
                }
            }
        }

    }

    return 0;
}

