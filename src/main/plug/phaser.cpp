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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>

#include <private/plugins/phaser.h>

/* The size of temporary buffer for audio processing */
#define BUFFER_SIZE         0x400U

namespace lsp
{
    namespace plugins
    {
        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::phaser_mono,
            &meta::phaser_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new phaser(meta);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //---------------------------------------------------------------------
        // Implementation
        phaser::phaser(const meta::plugin_t *meta):
            Module(meta)
        {
            pData           = NULL;
        }

        phaser::~phaser()
        {
            do_destroy();
        }

        void phaser::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // TODO
        }

        void phaser::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void phaser::do_destroy()
        {
            // TODO

            // Free previously allocated data chunk
            if (pData != NULL)
            {
                free_aligned(pData);
                pData       = NULL;
            }
        }

        void phaser::update_sample_rate(long sr)
        {
            // TODO
        }

        void phaser::update_settings()
        {
            // TODO
        }

        void phaser::process(size_t samples)
        {
            // TODO
        }

        void phaser::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            // TODO

            v->write("pData", pData);
        }

    } /* namespace plugins */
} /* namespace lsp */


