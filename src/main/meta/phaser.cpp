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
#define LSP_PLUGINS_PHASER_VERSION_MICRO       0

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
        //-------------------------------------------------------------------------
        // Plugin metadata

        // NOTE: Port identifiers should not be longer than 7 characters as it will overflow VST2 parameter name buffers
        static const port_t phaser_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            // Input controls
            BYPASS,

            PORTS_END
        };

        // NOTE: Port identifiers should not be longer than 7 characters as it will overflow VST2 parameter name buffers
        static const port_t phaser_stereo_ports[] =
        {
            // Input and output audio ports
            PORTS_STEREO_PLUGIN,

            // Input controls
            BYPASS,

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
            "", // TODO: provide ID of the video on YouTube
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
            E_DUMP_STATE,
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
            E_DUMP_STATE,
            phaser_stereo_ports,
            "effects/phaser.xml",
            "effects/phaser",
            stereo_plugin_port_groups,
            &phaser_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



