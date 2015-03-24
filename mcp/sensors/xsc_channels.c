#include <string.h>
#include "xsc_channels.h"

XSCChannelInfos xsc_channel_infos;

#define PI 3.141592653589793

static double channels_from_degrees(double angle)
{
    return angle*PI/180.;
}

static double channels_from_arcsec(double angle)
{
    return (angle/3600.)*PI/180.;
}

static void xsc_init_channel_info(XSCChannelInfo* info, const char* tx_name, XSCChannelForms form,
    double* rx_conversion, char tx_type, const char* slinger_type)
{
    strcpy(info->tx_name, tx_name);
    if (rx_conversion[0] == 0) {
        info->tx_m = 1.0;
        info->tx_b = 0.0;
    } else {
        info->tx_m = 1.0 / rx_conversion[0];
        info->tx_b = - rx_conversion[1] / rx_conversion[0];
    }
    info->rx_m = rx_conversion[0];
    info->rx_b = rx_conversion[1];
    info->form = form;
    info->tx_type = tx_type;
    strcpy(info->slinger_type, slinger_type);
}

static void init_ch_int(
    XSCChannelInfos* channel_infos, XSCChannelNames channel_name,
    const char* tx_name, XSCChannelForms form, double* rx_conversion, char tx_type,
    int value, const char* slinger_type)
{
    channel_infos->infos[channel_name].datatype = xN_int;
    channel_infos->infos[channel_name].initial_value.value_int = value;
    xsc_init_channel_info(&channel_infos->infos[channel_name], tx_name, form, rx_conversion, tx_type, slinger_type);
}

static void init_ch_double(
    XSCChannelInfos* channel_infos, XSCChannelNames channel_name,
    const char* tx_name, XSCChannelForms form, double* rx_conversion, char tx_type,
    double value, const char* slinger_type)
{
    channel_infos->infos[channel_name].datatype = xN_double;
    channel_infos->infos[channel_name].initial_value.value_double = value;
    xsc_init_channel_info(&channel_infos->infos[channel_name], tx_name, form, rx_conversion, tx_type, slinger_type);
}

static void init_ch_bool(
    XSCChannelInfos* channel_infos, XSCChannelNames channel_name,
    const char* tx_name, XSCChannelForms form, double* rx_conversion, char tx_type,
    bool value, const char* slinger_type)
{
    channel_infos->infos[channel_name].datatype = xN_bool;
    channel_infos->infos[channel_name].initial_value.value_bool = value;
    xsc_init_channel_info(&channel_infos->infos[channel_name], tx_name, form, rx_conversion, tx_type, slinger_type);
}

void xsc_init_channels(XSCChannelInfos* xsc_channel_infos)
{
    XSCChannelInfos* ci = xsc_channel_infos;

    double narrow_max = 65536.0;
    double narrow_signed_max = 32768.0;
    double wide_max = 4294967296.0;
    double identity[2] = {1.0, 0.0};
    double disk_gb[2] = {2000.0/narrow_max, 0.0};
    double temperature[2] = {473.15/narrow_max, -273.15};
    double pressure[2] = {5.0/narrow_max, 0.0};
    double gain_db[2] = {128.0/narrow_signed_max, 0.0};
    double angle_wide_2pi[2] = {channels_from_degrees(720.0)/wide_max, -channels_from_degrees(360.0)};
    double iplate[2] = {channels_from_arcsec(20.0)/narrow_max, 0.0};
    double stats_depth[2] = {(8*4096.0)/narrow_max, -1.0};
    double stats_1000[2] = {1001.0/narrow_max, -1.0};
    double bounded_1s[2] = {2.0/(narrow_max-1.0), -1.0};

    init_ch_int(    ci, xN_ctr_stars                   , "ctr_stars"                   , xN_wide_fast,   identity,       'S', -1,    "periodic");
    init_ch_int(    ci, xN_stars_run_time              , "stars_run_time"              , xN_wide_slow,   identity,       'U', 0,     "noisy");
    init_ch_double( ci, xN_hk_temp_lens                , "hk_temp_lens"                , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_temp_comp                , "hk_temp_comp"                , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_temp_vessel              , "hk_temp_vessel"              , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_temp_plate               , "hk_temp_plate"               , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_temp_dcdc                , "hk_temp_dcdc"                , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_temp_flange              , "hk_temp_flange"              , xN_narrow_slow, temperature,    'u', 100.0, "noisy");
    init_ch_double( ci, xN_hk_pressure                 , "hk_pressure"                 , xN_narrow_slow, pressure,       'u', 0.0,   "noisy");
    init_ch_double( ci, xN_hk_disk                     , "hk_disk"                     , xN_narrow_slow, disk_gb,        'u', 0.0,   "changing");
    init_ch_bool(   ci, xN_cam_gain_valid              , "cam_gain_valid"              , xN_narrow_slow, identity,       'u', false, "periodic");
    init_ch_double( ci, xN_cam_gain_db                 , "cam_gain_db"                 , xN_narrow_slow, gain_db,        's', -1.0,  "periodic");
    init_ch_int(    ci, xN_lens_focus                  , "lens_focus"                  , xN_narrow_slow, identity,       's', -1,    "periodic");
    init_ch_int(    ci, xN_lens_aperture               , "lens_aperture"               , xN_narrow_slow, identity,       's', -1,    "periodic");
    init_ch_int(    ci, xN_image_ctr_stars             , "image_ctr_stars"             , xN_wide_fast,   identity,       'S', -1,    "periodic");
    init_ch_int(    ci, xN_image_ctr_fcp               , "image_ctr_fcp"               , xN_wide_fast,   identity,       'S', -1,    "periodic");
    init_ch_int(    ci, xN_image_num_exposures         , "image_num_exposures"         , xN_narrow_slow, identity,       's', -1,    "periodic");
    init_ch_double( ci, xN_image_stats_mean            , "image_stats_mean"            , xN_narrow_slow, stats_depth,    'u', -1.0,   "periodic");
    init_ch_double( ci, xN_image_stats_noise           , "image_stats_noise"           , xN_narrow_slow, stats_1000,     'u', -1.0,  "periodic");
    init_ch_double( ci, xN_image_stats_gaindb          , "image_stats_gaindb"          , xN_narrow_slow, stats_1000,     'u', -1.0,  "periodic");
    init_ch_int(    ci, xN_image_stats_num_px_sat      , "image_stats_num_px_sat"      , xN_narrow_slow, identity,       's', -1,    "periodic");
    init_ch_double( ci, xN_image_stats_frac_px_sat     , "image_stats_frac_px_sat"     , xN_narrow_slow, bounded_1s,     'u', -1.0,  "periodic");
    init_ch_bool(   ci, xN_image_afocus_metric_valid   , "image_afocus_metric_valid"   , xN_narrow_slow, identity,       'u', false, "periodic");
    init_ch_double( ci, xN_image_afocus_metric         , "image_afocus_metric"         , xN_narrow_slow, stats_depth,    'u', 0.0,   "periodic");
    init_ch_bool(   ci, xN_image_eq_valid              , "image_eq_valid"              , xN_narrow_slow, identity,       'u', false, "periodic");
    init_ch_double( ci, xN_image_eq_ra                 , "image_eq_ra"                 , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_dec                , "image_eq_dec"                , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_roll               , "image_eq_roll"               , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_sigma_ra           , "image_eq_sigma_ra"           , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_sigma_dec          , "image_eq_sigma_dec"          , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_sigma_roll         , "image_eq_sigma_roll"         , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_sigma_pointing     , "image_eq_sigma_pointing"     , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_eq_iplate             , "image_eq_iplate"             , xN_narrow_slow, iplate,         'u', 0.0,   "periodic");
    init_ch_bool(   ci, xN_image_hor_valid             , "image_hor_valid"             , xN_narrow_slow, identity,       'u', false, "periodic");
    init_ch_double( ci, xN_image_hor_az                , "image_hor_az"                , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_el                , "image_hor_el"                , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_roll              , "image_hor_roll"              , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_sigma_az          , "image_hor_sigma_az"          , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_sigma_el          , "image_hor_sigma_el"          , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_sigma_roll        , "image_hor_sigma_roll"        , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_sigma_pointing    , "image_hor_sigma_pointing"    , xN_wide_slow,   angle_wide_2pi, 'U', 0.0,   "periodic");
    init_ch_double( ci, xN_image_hor_iplate            , "image_hor_iplate"            , xN_narrow_slow, iplate,         'u', 0.0,   "periodic");
    init_ch_int(    ci, xN_image_num_blobs_found       , "image_num_blobs_found"       , xN_narrow_slow, identity,       's', -1,    "periodic");
    init_ch_int(    ci, xN_image_num_blobs_matched     , "image_num_blobs_matched"     , xN_narrow_slow, identity,       's', -1,    "periodic");

}

