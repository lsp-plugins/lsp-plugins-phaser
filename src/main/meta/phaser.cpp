/*
 * Copyright (C) 2025 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2025 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-phaser
 * Created on: 10 мар 2025 г.
 *
 * lsp-plugins-phaser is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-phaser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-phaser. If not, see <https://www.gnu.org/licenses/>.
 */

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/phaser.h>

#define LSP_PLUGINS_PHASER_VERSION_MAJOR       1
#define LSP_PLUGINS_PHASER_VERSION_MINOR       0
#define LSP_PLUGINS_PHASER_VERSION_MICRO       1

#define LSP_PLUGINS_PHASER_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_PHASER_VERSION_MAJOR, \
        LSP_PLUGINS_PHASER_VERSION_MINOR, \
        LSP_PLUGINS_PHASER_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        #define LFO_LIST \
            { "Triangular",             "phaser.osc.triangular"             }, \
            { "Sine",                   "phaser.osc.sine"                   }, \
            { "Stepped Sine",           "phaser.osc.stepped_sine"           }, \
            { "Cubic",                  "phaser.osc.cubic"                  }, \
            { "Stepped Cubic",          "phaser.osc.stepped_cubic"          }, \
            { "Parabolic",              "phaser.osc.parabolic"              }, \
            { "Reverse Parabolic",      "phaser.osc.reverse_parabolic"      }, \
            { "Logarithmic",            "phaser.osc.logarithmic"            }, \
            { "Reverse Logarithmic",    "phaser.osc.reverse_logarithmic"    }, \
            { "Square Root",            "phaser.osc.square_root"            }, \
            { "Reverse Square Root",    "phaser.osc.reverse_square_root"    }, \
            { "Circular",               "phaser.osc.circular"               }, \
            { "Reverse Circular",       "phaser.osc.reverse_circular"       }, \
            { NULL, NULL }

        static const port_item_t osc_functions[] =
        {
            LFO_LIST
        };

        static const port_item_t additional_osc_functions[] =
        {
            { "Same",               "phaser.osc.same"                   },
            LFO_LIST
        };

        static const port_item_t osc_periods[] =
        {
            { "Full",                   "phaser.period.full"                },
            { "First",                  "phaser.period.first"               },
            { "Last",                   "phaser.period.last"                },
            { NULL, NULL }
        };

        static const port_item_t rate_type[] =
        {
            { "Rate",                   "phaser.rate.rate"                  },
            { "Tempo",                  "phaser.rate.tempo"                 },
            { "Static",                 "phaser.rate.static"                },
            { NULL, NULL }
        };

        static const port_item_t filters_list[] =
        {
            { "1",                      NULL                                },
            { "2",                      NULL                                },
            { "3",                      NULL                                },
            { "4",                      NULL                                },
            { "5",                      NULL                                },
            { "6",                      NULL                                },
            { "7",                      NULL                                },
            { "8",                      NULL                                },
            { NULL,                     NULL                                }
        };

        static const port_item_t filter_slopes[] =
        {
            { "off",        "eq.slope.off"      },
            { "12 dB/oct",  "eq.slope.12dbo"    },
            { "24 dB/oct",  "eq.slope.24dbo"    },
            { "36 dB/oct",  "eq.slope.36dbo"    },
            { NULL, NULL }
        };

        //-------------------------------------------------------------------------
        // Plugin metadata

        #define FILTER_METER_MONO(id, label) \
            METER("fmp" id, "Filter meter" label " phase", U_DEG, phaser::PHASE), \
            METER("fms" id, "Filter meter" label " shift", U_NONE, phaser::SHIFT), \
            METER("fmf" id, "Filter meter" label " frequency", U_MSEC, phaser::LFO_FREQ)

        #define FILTER_METER_STEREO(id, label) \
            FILTER_METER_MONO(id "l", label " left"), \
            FILTER_METER_MONO(id "r", label " right")

        #define TEMPO_SETTINGS \
            LOG_CONTROL("rate", "Rate", "Rate", U_HZ, phaser::RATE), \
            AMP_GAIN10("depth", "Depth", "Depth", GAIN_AMP_0_DB), \
            CONTROL("frac", "Time fraction", "Frac", U_BAR, phaser::FRACTION), \
            CONTROL("denom", "Time fraction denominator", "Denom", U_BAR, phaser::DENOMINATOR), \
            CONTROL("tempo", "Tempo", "Tempo", U_BPM, phaser::TEMPO), \
            SWITCH("sync", "Tempo sync", "Sync tempo", 0.0f), \
            COMBO("time", "Time computing method", "Method", 0, rate_type), \
            TRIGGER("reset", "Reset phase to initial value", "Reset")

        #define FEEDBACK_SETTINGS \
            SWITCH("fb_on", "Feedback on", "Feed On", 0), \
            CONTROL("fgain", "Feedback gain", "Feed gain", U_GAIN_AMP, phaser::FEEDBACK_GAIN), \
            CONTROL("fdelay", "Feedback delay", "Feed delay", U_MSEC, phaser::FEEDBACK_DELAY), \
            SWITCH("fphase", "Feedback phase switch", "Feed phase", 0.0f)

        static const port_t phaser_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            // Bypass
            BYPASS,

            // Operating modes
            SWITCH("sphase", "Signal phase switch", "Phase", 0.0f),
            COMBO("hpm", "High-pass filter mode", "HPF mode", 0, filter_slopes),
            LOG_CONTROL("hpf", "High-pass filter frequency", "HPF freq", U_HZ, phaser::HPF),
            COMBO("lpm", "Low-pass filter mode", "LPF mode", 0, filter_slopes),
            LOG_CONTROL("lpf", "Low-pass filter frequency", "LPF freq", U_HZ, phaser::LPF),

            // Tempo/rate controls
            TEMPO_SETTINGS,

            // LFO settings
            COMBO("filters", "Number of filters", "Num filters", phaser::FILTERS_DFL - phaser::FILTERS_MIN, filters_list),
            LOG_CONTROL("qfactor", "Filter quality (Q factor)", "Q Factor", U_NONE, phaser::FILTER_QUALITY),
            CONTROL("xfade", "Crossfade", "Xfade", U_PERCENT, phaser::CROSSFADE),
            COMBO("type", "LFO type", "LFO type", 1, osc_functions),
            COMBO("period", "LFO period", "LFO period", 0, osc_periods),
            CONTROL("lo", "LFO overlap", "LFO overlap", U_PERCENT, phaser::OVERLAP),
            LOG_CONTROL_DFL("lfs", "LFO frequency start", "LFO start", U_HZ, phaser::LFO_FREQ, phaser::LFO_FREQ_START),
            LOG_CONTROL_DFL("lfe", "LFO frequency end", "LFO end", U_HZ, phaser::LFO_FREQ, phaser::LFO_FREQ_END),
            CYC_CONTROL("lip", "LFO initial phase", "LFO phase", U_DEG, phaser::PHASE),
            CYC_CONTROL("lfp", "Inter-filter phase range", "Inter-phase", U_DEG, phaser::FILTER_PHASE),
            MESH("lgr", "LFO graph", phaser::FILTERS_MAX + 1, phaser::LFO_MESH_SIZE),

            // Feedback chain
            FEEDBACK_SETTINGS,

            // Loudness control
            IN_GAIN,
            DRY_GAIN(GAIN_AMP_M_INF_DB),
            WET_GAIN(GAIN_AMP_0_DB),
            DRYWET(100.0f),
            OUT_GAIN,

            // Filter meters
            FILTER_METER_MONO("_1", " 1"),
            FILTER_METER_MONO("_2", " 2"),
            FILTER_METER_MONO("_3", " 3"),
            FILTER_METER_MONO("_4", " 4"),
            FILTER_METER_MONO("_5", " 5"),
            FILTER_METER_MONO("_6", " 6"),
            FILTER_METER_MONO("_7", " 7"),
            FILTER_METER_MONO("_8", " 8"),

            // Gain meters
            METER_GAIN("min", "Input gain", GAIN_AMP_P_48_DB),
            METER_GAIN("mout", "Output gain", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const port_t phaser_stereo_ports[] =
        {
            // Input and output audio ports
            PORTS_STEREO_PLUGIN,

            // Bypass
            BYPASS,

            // Operating modes
            SWITCH("mono", "Test for mono compatibility", "Mono", 0),
            SWITCH("ms", "Mid/Side mode switch", "M/S mode", 0.0f),
            SWITCH("sphase", "Signal phase switch", "Phase", 0.0f),
            COMBO("hpm", "High-pass filter mode", "HPF mode", 0, filter_slopes),
            LOG_CONTROL("hpf", "High-pass filter frequency", "HPF freq", U_HZ, phaser::HPF),
            COMBO("lpm", "Low-pass filter mode", "LPF mode", 0, filter_slopes),
            LOG_CONTROL("lpf", "Low-pass filter frequency", "LPF freq", U_HZ, phaser::LPF),

            // Tempo/rate controls
            TEMPO_SETTINGS,

            // LFO settings
            COMBO("filters", "Number of filters", "Num filters", phaser::FILTERS_DFL - phaser::FILTERS_MIN, filters_list),
            LOG_CONTROL("qfactor", "Filter quality (Q factor)", "Q Factor", U_NONE, phaser::FILTER_QUALITY),
            CONTROL("xfade", "Crossfade", "Xfade", U_PERCENT, phaser::CROSSFADE),
            COMBO("type", "LFO type", "LFO type", 1, osc_functions),
            COMBO("period", "LFO period", "LFO period", 0, osc_periods),
            COMBO("atype", "Additional LFO type", "LFO2 type", 0, additional_osc_functions),
            COMBO("aperiod", "Additional LFO period", "LFO2 period", 0, osc_periods),
            CONTROL("lo", "LFO overlap", "LFO overlap", U_PERCENT, phaser::OVERLAP),
            LOG_CONTROL_DFL("lfs", "LFO frequency start", "LFO start", U_HZ, phaser::LFO_FREQ, phaser::LFO_FREQ_START),
            LOG_CONTROL_DFL("lfe", "LFO frequency end", "LFO end", U_HZ, phaser::LFO_FREQ, phaser::LFO_FREQ_END),
            CYC_CONTROL("lip", "LFO initial phase", "LFO phase", U_DEG, phaser::PHASE),
            CYC_CONTROL("lfp", "Inter-filter phase range", "Inter-phase", U_DEG, phaser::FILTER_PHASE),
            CYC_CONTROL("lcp", "Inter-channel phase", "Chan phase", U_DEG, phaser::CHANNEL_PHASE),
            MESH("lgr", "LFO graph", phaser::FILTERS_MAX * 2 + 1, phaser::LFO_MESH_SIZE),

            // Feedback chain
            FEEDBACK_SETTINGS,

            // Loudness control
            IN_GAIN,
            DRY_GAIN(GAIN_AMP_M_INF_DB),
            WET_GAIN(GAIN_AMP_0_DB),
            DRYWET(100.0f),
            OUT_GAIN,

            // Filter meters
            FILTER_METER_STEREO("_1", " 1"),
            FILTER_METER_STEREO("_2", " 2"),
            FILTER_METER_STEREO("_3", " 3"),
            FILTER_METER_STEREO("_4", " 4"),
            FILTER_METER_STEREO("_5", " 5"),
            FILTER_METER_STEREO("_6", " 6"),
            FILTER_METER_STEREO("_7", " 7"),
            FILTER_METER_STEREO("_8", " 8"),

            // Gain meters
            METER_GAIN("min_l", "Input gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_l", "Output gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("min_r", "Input gain right",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_r", "Output gain right", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_PHASER, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_PHASER, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_PHASER, CF_STEREO, -1 };

        const meta::bundle_t phaser_bundle =
        {
            "phaser",
            "Phaser",
            B_EFFECTS,
            "iqiXtMQckak",
            "This plugin allows to simpulate multi-stage phaser effect"
        };

        const plugin_t phaser_mono =
        {
            "Phaser Mono",
            "Phaser Mono",
            "Phaser Mono",
            "PH1M",
            &developers::v_sadovnikov,
            "phaser_mono",
            {
                LSP_LV2_URI("phaser_mono"),
                LSP_LV2UI_URI("phaser_mono"),
                "ph1m",
                LSP_VST3_UID("ph1m    ph1m"),
                LSP_VST3UI_UID("ph1m    ph1m"),
                LSP_LADSPA_PHASER_BASE + 0,
                LSP_LADSPA_URI("phaser_mono"),
                LSP_CLAP_URI("phaser_mono"),
                LSP_GST_UID("phaser_mono"),
            },
            LSP_PLUGINS_PHASER_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            phaser_mono_ports,
            "effects/phaser.xml",
            "effects/phaser",
            mono_plugin_port_groups,
            &phaser_bundle
        };

        const plugin_t phaser_stereo =
        {
            "Phaser Stereo",
            "Phaser Stereo",
            "Phaser Stereo",
            "PH1S",
            &developers::v_sadovnikov,
            "phaser_stereo",
            {
                LSP_LV2_URI("phaser_stereo"),
                LSP_LV2UI_URI("phaser_stereo"),
                "ph1s",
                LSP_VST3_UID("ph1s    ph1s"),
                LSP_VST3UI_UID("ph1s    ph1s"),
                LSP_LADSPA_PHASER_BASE + 1,
                LSP_LADSPA_URI("phaser_stereo"),
                LSP_CLAP_URI("phaser_stereo"),
                LSP_GST_UID("phaser_stereo"),
            },
            LSP_PLUGINS_PHASER_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            phaser_stereo_ports,
            "effects/phaser.xml",
            "effects/phaser",
            stereo_plugin_port_groups,
            &phaser_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



