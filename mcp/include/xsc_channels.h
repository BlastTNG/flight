#ifdef _MSC_VER
    #ifndef __cplusplus
        typedef int bool;
        #define true 1
        #define false 0
    #endif
#else
    #include <stdbool.h>
#endif

typedef enum
{
    xN_ctr_stars = 0,
    xN_stars_run_time,

    xN_hk_temp_lens,
    xN_hk_temp_comp,
    xN_hk_temp_vessel,
    xN_hk_temp_plate,
    xN_hk_temp_dcdc,
    xN_hk_temp_flange,
    xN_hk_pressure,
    xN_hk_disk,

    xN_cam_gain_valid,
    xN_cam_gain_db,

    xN_lens_focus,
    xN_lens_aperture,

    xN_image_ctr_stars,
    xN_image_ctr_fcp,
    xN_image_num_exposures,
    xN_image_stats_mean,
    xN_image_stats_noise,
    xN_image_stats_gaindb,
    xN_image_stats_num_px_sat,
    xN_image_stats_frac_px_sat,
    xN_image_afocus_metric_valid,
    xN_image_afocus_metric,
    xN_image_eq_valid,
    xN_image_eq_ra,
    xN_image_eq_dec,
    xN_image_eq_roll,
    xN_image_eq_sigma_ra,
    xN_image_eq_sigma_dec,
    xN_image_eq_sigma_roll,
    xN_image_eq_sigma_pointing,
    xN_image_eq_iplate,
    xN_image_hor_valid,
    xN_image_hor_az,
    xN_image_hor_el,
    xN_image_hor_roll,
    xN_image_hor_sigma_az,
    xN_image_hor_sigma_el,
    xN_image_hor_sigma_roll,
    xN_image_hor_sigma_pointing,
    xN_image_hor_iplate,
    xN_image_num_blobs_found,
    xN_image_num_blobs_matched,

    xN_num_channels
}
XSCChannelNames;

typedef struct
{
    union
    {
        int value_int;
        double value_double;
        bool value_bool;
    };
}
XSCChannel;

typedef enum
{
    xN_int = 0,
    xN_double,
    xN_bool
}
XSCChannelDatatypes;

typedef enum
{
    xN_narrow_slow = 0,
    xN_narrow_fast,
    xN_wide_slow,
    xN_wide_fast
}
XSCChannelForms;

typedef struct
{
    XSCChannelDatatypes datatype;
    XSCChannel initial_value;
    char tx_name[32];
    double tx_m;
    double tx_b;
    double rx_m;
    double rx_b;
    XSCChannelForms form;
    char tx_type;
    char slinger_type[32];
}
XSCChannelInfo;

typedef struct XSCChannelInfos
{
    XSCChannelInfo infos[xN_num_channels];
}
XSCChannelInfos;

void xsc_init_channels(XSCChannelInfos* xsc_channel_infos);

