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
        dspu::lfo::function_t phaser::all_lfo_functions[] =
        {
            dspu::lfo::triangular,
            dspu::lfo::sine,
            dspu::lfo::step_sine,
            dspu::lfo::cubic,
            dspu::lfo::step_cubic,
            dspu::lfo::parabolic,
            dspu::lfo::rev_parabolic,
            dspu::lfo::logarithmic,
            dspu::lfo::rev_logarithmic,
            dspu::lfo::sqrt,
            dspu::lfo::rev_sqrt,
            dspu::lfo::circular,
            dspu::lfo::rev_circular
        };


        phaser::phaser(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels           = 0;
            nFilters            = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Cleanup data
            vChannels           = NULL;

            sLfo.nType          = -1;
            sLfo.nPeriod        = -1;
            sLfo.fOverlap       = 0.0f;
            sLfo.fMinFreq       = meta::phaser::LFO_FREQ_START;
            sLfo.fMaxFreq       = meta::phaser::LFO_FREQ_END;
            sLfo.nOldInitPhase  = 0.0f;
            sLfo.nInitPhase     = 0;
            sLfo.fIFilterPhase  = 0.0f;
            sLfo.fIChanPhase    = 0;
            sLfo.fArg[0]        = 0;
            sLfo.fArg[1]        = 0;

            sLfo.pFunc          = NULL;
            sLfo.vLfoMesh       = NULL;

            sLfo.bSyncMesh      = true;

            sLfo.pType          = NULL;
            sLfo.pPeriod        = NULL;
            sLfo.pOverlap       = NULL;
            sLfo.pMinFreq       = NULL;
            sLfo.pMaxFreq       = NULL;
            sLfo.pInitPhase     = NULL;
            sLfo.pIFilterPhase  = NULL;
            sLfo.pIChannelPhase = NULL;
            sLfo.pMesh          = NULL;

            vBuffer             = NULL;
            vLfoPhase           = NULL;

            // Ports
            pBypass             = NULL;
            pMono               = NULL;
            pMS                 = NULL;
            pInvPhase           = NULL;
            pHpfMode            = NULL;
            pHpfFreq            = NULL;
            pLpfMode            = NULL;
            pLpfFreq            = NULL;

            pRate               = NULL;
            pFraction           = NULL;
            pTempo              = NULL;
            pTempoSync          = NULL;
            pTimeMode           = NULL;
            pReset              = NULL;

            pFilters            = NULL;
            pCrossfade          = NULL;
            pCrossfadeType      = NULL;

            pFeedOn             = NULL;
            pFeedGain           = NULL;
            pFeedDelay          = NULL;
            pFeedPhase          = NULL;

            pInGain             = NULL;
            pDryGain            = NULL;
            pWetGain            = NULL;
            pDryWet             = NULL;
            pOutGain            = NULL;

            pData               = NULL;
        }

        phaser::~phaser()
        {
            do_destroy();
        }

        void phaser::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Estimate the number of bytes to allocate
            const size_t szof_channels  = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            const size_t buf_sz         = BUFFER_SIZE * sizeof(float);
            const size_t mesh_buf_sz    = align_size(meta::phaser::LFO_MESH_SIZE * sizeof(float), OPTIMAL_ALIGN);
            const size_t to_alloc       =
                szof_channels +         // vChannels
                buf_sz +                // vBuffer
                mesh_buf_sz +           // vLfoPhase
                mesh_buf_sz +           // lfo_t::vLfoMesh
                nChannels * buf_sz;     // channel_t::vBuffer

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, to_alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;
            lsp_guard_assert(uint8_t *save   = ptr);

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);
            vBuffer                 = advance_ptr_bytes<float>(ptr, buf_sz);
            vLfoPhase               = advance_ptr_bytes<float>(ptr, mesh_buf_sz);
            sLfo.vLfoMesh           = advance_ptr_bytes<float>(ptr, mesh_buf_sz);

            // Initialize channels
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.construct();
                c->sEq.construct();

                c->sEq.init(2, 0);
                c->sEq.set_mode(dspu::EQM_IIR);

                for (size_t j=0; j<meta::phaser::FILTERS_MAX; ++j)
                {
                    filter_t *f             = &c->vFilters[j];

                    f->nPhase               = 0;
                    f->fNormShift           = 0.0f;
                    f->fNormScale           = 0.0f;
                    f->fOutPhase            = 0.0f;
                    f->fOutShift            = 0.0f;
                    f->fOutFreq             = 0.0f;

                    f->pPhase               = NULL;
                    f->pShift               = NULL;
                    f->pOutFreq             = NULL;
                }

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vBuffer              = advance_ptr_bytes<float>(ptr, buf_sz);

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pInLevel             = NULL;
                c->pOutLevel            = NULL;
            }

            lsp_assert(ptr <= &save[to_alloc]);

            // Bind ports
            size_t port_id      = 0;

            // Bind I/O ports
            lsp_trace("Binding I/O ports");
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pIn);
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pOut);

            // Bind bypass
            lsp_trace("Binding bypass ports");
            BIND_PORT(pBypass);

            // Operating modes
            lsp_trace("Binding operating modes");
            if (nChannels > 1)
            {
                BIND_PORT(pMono);              // Mono compatibility test
                BIND_PORT(pMS);                // Mid/Side switch
            }
            BIND_PORT(pInvPhase);
            BIND_PORT(pHpfMode);
            BIND_PORT(pHpfFreq);
            BIND_PORT(pLpfMode);
            BIND_PORT(pLpfFreq);

            // Tempo/rate controls
            lsp_trace("Binding tempo/rate controls");
            BIND_PORT(pRate);
            BIND_PORT(pFraction);
            SKIP_PORT("Denominator");   // Skip denominator
            BIND_PORT(pTempo);
            BIND_PORT(pTempoSync);
            BIND_PORT(pTimeMode);
            BIND_PORT(pReset);

            // LFO settings
            lsp_trace("Binding LFO settings");
            BIND_PORT(pFilters);
            BIND_PORT(pCrossfade);
            BIND_PORT(pCrossfadeType);

            BIND_PORT(sLfo.pType);
            BIND_PORT(sLfo.pPeriod);
            BIND_PORT(sLfo.pOverlap);
            BIND_PORT(sLfo.pMinFreq);
            BIND_PORT(sLfo.pMaxFreq);
            BIND_PORT(sLfo.pInitPhase);
            BIND_PORT(sLfo.pIFilterPhase);
            if (nChannels > 1)
                BIND_PORT(sLfo.pIChannelPhase);
            BIND_PORT(sLfo.pMesh);

            // Feedback settings
            lsp_trace("Binding feedback settings");
            BIND_PORT(pFeedOn);
            BIND_PORT(pFeedGain);
            BIND_PORT(pFeedDelay);
            BIND_PORT(pFeedPhase);

            // Loudness control settings
            lsp_trace("Binding loudness control settings");
            BIND_PORT(pInGain);
            BIND_PORT(pDryGain);
            BIND_PORT(pWetGain);
            BIND_PORT(pDryWet);
            BIND_PORT(pOutGain);

            // Filter meters
            for (size_t i=0; i<meta::phaser::FILTERS_MAX; ++i)
            {
                for (size_t j=0; j<nChannels; ++j)
                {
                    filter_t *f             = &vChannels[j].vFilters[i];

                    BIND_PORT(f->pPhase);
                    BIND_PORT(f->pShift);
                    BIND_PORT(f->pOutFreq);
                }
            }

            // Bind signal meters
            lsp_trace("Binding signal meters");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                BIND_PORT(c->pInLevel);
                BIND_PORT(c->pOutLevel);
            }

            // Fill LFO phase data
            float phase_k           = 360.0f / (meta::phaser::LFO_MESH_SIZE - 1);
            for (size_t i=0; i<meta::phaser::LFO_MESH_SIZE; ++i)
                vLfoPhase[i]            = i * phase_k;
        }

        void phaser::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void phaser::do_destroy()
        {
            // Destroy channels
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    c->sBypass.destroy();
                    c->sEq.destroy();
                }
                vChannels   = NULL;
            }

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


