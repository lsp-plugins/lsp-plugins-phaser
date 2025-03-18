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

namespace lsp
{
    namespace plugins
    {
        static constexpr uint32_t   BUFFER_SIZE             = 0x400;
        static constexpr uint32_t   PHASE_MAX               = 0x80000000;
        static constexpr uint32_t   PHASE_MASK              = PHASE_MAX - 1;
        static constexpr float      PHASE_COEFF             = 1.0f / float(PHASE_MAX);

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
            sLfo.fOldMinFreq    = meta::phaser::LFO_FREQ_START;
            sLfo.fMinFreq       = meta::phaser::LFO_FREQ_START;
            sLfo.fOldMaxFreq    = meta::phaser::LFO_FREQ_END;
            sLfo.fMaxFreq       = meta::phaser::LFO_FREQ_END;
            sLfo.nOldInitPhase  = 0.0f;
            sLfo.nInitPhase     = 0;
            sLfo.nPhase         = 0;
            sLfo.nOldPhaseStep  = 0;
            sLfo.nPhaseStep     = 0;
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

            fRate               = 0.0f;
            nCrossfade          = 0;
            fCrossfade          = 0.0f;
            pCrossfadeFunc      = NULL;
            fOldInGain          = GAIN_AMP_0_DB;
            fInGain             = fOldInGain;
            fOldDryGain         = GAIN_AMP_M_INF_DB;
            fDryGain            = fOldDryGain;
            fOldWetGain         = GAIN_AMP_0_DB;
            fWetGain            = fOldWetGain;
            fOldFeedGain        = GAIN_AMP_M_INF_DB;
            fFeedGain           = GAIN_AMP_M_INF_DB;
            nOldFeedDelay       = 0;
            nFeedDelay          = 0;

            bMS                 = false;
            bMono               = false;
            bUpdateFilters      = true;

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
                c->sFeedback.construct();
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
                    c->sFeedback.destroy();
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

        inline uint32_t phaser::phase_to_int(float phase)
        {
            if (phase >= 360.0f)
                phase      -= 360.0f;
            return float(PHASE_MAX) * (phase / 360.0f);
        }

        inline float phaser::lerp(float o_value, float n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        inline float phaser::qlerp(float o_value, float n_value, float k)
        {
            return o_value * sqrtf(1.0f - k) + n_value * sqrtf(k);
        }

        inline int32_t phaser::ilerp(int32_t o_value, int32_t n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        inline int32_t phaser::elerp(int32_t o_value, int32_t n_value, float k)
        {
            return o_value * expf(logf(n_value/o_value) * k);
        }

        void phaser::update_sample_rate(long sr)
        {
            plug::Module::update_sample_rate(sr);

            // Update sample rate for the bypass processors
            size_t max_feedback = dspu::millis_to_samples(sr, meta::phaser::FEEDBACK_DELAY_MAX);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                c->sBypass.init(sr);
                c->sFeedback.init(max_feedback + BUFFER_SIZE*2);
                c->sEq.set_sample_rate(sr);
            }
        }

        void phaser::update_settings()
        {
            // Pre-compute several attributes
            const float in_gain     = pInGain->value();
            const float out_gain    = pOutGain->value();
            const bool bypass       = pBypass->value() >= 0.5f;
            bool fb_on              = pFeedOn->value() >= 0.5f;
            float feed_gain         = (fb_on) ? pFeedGain->value() : 0.0f;
            const bool mid_side     = (pMS != NULL) ? pMS->value() >= 0.5f : false;
            float crossfade         = pCrossfade->value() * 0.01f;

            // Compute LFO rate and phase
            float rate              = 0.0f;
            uint32_t mode           = pTimeMode->value();
            sLfo.nOldPhaseStep      = sLfo.nPhaseStep;

            switch (mode)
            {
                case meta::phaser::MODE_TEMPO:
                {
                    float tempo             = (pTempoSync->value() >= 0.5f) ? pWrapper->position()->beatsPerMinute : pTempo->value();
                    rate                    = lsp_limit(
                        dspu::time_signature_to_frequency(pFraction->value(), tempo),
                        meta::phaser::RATE_MIN,
                        meta::phaser::RATE_MAX) / float(fSampleRate);

                    sLfo.nPhaseStep         = float(PHASE_MAX) * rate;
                    break;
                }

                case meta::phaser::MODE_STATIC:
                {
                    rate                    = fRate;
                    sLfo.nPhaseStep         = 0;
                    sLfo.nPhase             = 0;
                    break;
                }

                case meta::phaser::MODE_RATE:
                default:
                    rate                    = pRate->value() / float(fSampleRate);
                    sLfo.nPhaseStep         = float(PHASE_MAX) * rate;
                    break;
            }

            if (fRate != rate)
                bUpdateFilters              = true;

            // Update common parameters
            const float dry_gain    = pDryGain->value();
            const float wet_gain    = (pInvPhase->value() < 0.5f) ? pWetGain->value() : -pWetGain->value();
            const float drywet      = pDryWet->value() * 0.01f;

            fOldInGain              = fInGain;
            fOldDryGain             = fDryGain;
            fOldWetGain             = fWetGain;
            fInGain                 = in_gain;
            fDryGain                = (dry_gain * drywet + 1.0f - drywet) * out_gain;
            fWetGain                = wet_gain * drywet * out_gain;
            nOldFeedDelay           = nFeedDelay;
            nFeedDelay              = dspu::millis_to_samples(fSampleRate, pFeedDelay->value());
            fOldFeedGain            = fFeedGain;
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            nCrossfade              = float(PHASE_MAX) * crossfade * 2.0f;
            fCrossfade              = PHASE_COEFF * (1.0f - crossfade);
            pCrossfadeFunc          = (int(pCrossfadeType->value()) == 0) ? lerp : qlerp;

            // LFO setup
            const size_t filters    = lsp_min(pFilters->value() + 1, meta::phaser::FILTERS_MAX);
            const float if_phase    = sLfo.pIFilterPhase->value();
            const float ichan_phase = (sLfo.pIChannelPhase != NULL) ? sLfo.pIChannelPhase->value() : 0.0f;
            const float overlap     = sLfo.pOverlap->value() * 0.01f;

            if (filters != nFilters)
            {
                nFilters                = filters;
                sLfo.bSyncMesh          = true;
                bUpdateFilters          = true;
            }

            if (sLfo.fOverlap != overlap)
            {
                sLfo.fOverlap           = overlap;
                sLfo.bSyncMesh          = true;
                bUpdateFilters          = true;
            }

            if ((sLfo.fIFilterPhase != if_phase) ||
                (sLfo.fIChanPhase != ichan_phase))
            {
                sLfo.fIFilterPhase      = if_phase;
                sLfo.fIChanPhase        = ichan_phase;
                bUpdateFilters          = true;
            }

            // Update filters if required
            if (bUpdateFilters)
            {
                bUpdateFilters          = false;

                const float p_step      = sLfo.fIFilterPhase / float(nFilters);
                const float ovl_width   = lerp(1.0f / nFilters, 1.0f, sLfo.fOverlap);
                const float ovl_step    = (nFilters > 1) ? (1.0f - ovl_width) / (nFilters - 1) : 0.0f;

                for (size_t i=0; i < nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];
                    const float base_phase  = sLfo.fIChanPhase * i;

                    for (size_t j=0; j < nFilters; ++j)
                    {
                        const float v_shift     = j * ovl_step;

                        filter_t *f             = &c->vFilters[j];
                        f->nPhase               = phase_to_int(base_phase + p_step * j);
                        f->fNormShift           = v_shift;
                        f->fNormScale           = ovl_width;
                    }
                }
            }

            // Update LFO preferences
            const size_t lfo_type   = size_t(sLfo.pType->value());
            const size_t lfo_period = size_t(sLfo.pPeriod->value());

            // The form of the LFO has changed?
            if ((lfo_type != sLfo.nType) || (lfo_period != sLfo.nPeriod))
            {
                sLfo.nType              = lfo_type;
                sLfo.nPeriod            = lfo_period;
                sLfo.pFunc              = all_lfo_functions[lfo_type];
                sLfo.bSyncMesh          = true;

                // Select the function coefficients
                switch (lfo_period)
                {
                    case meta::phaser::OSC_FIRST:
                        sLfo.fArg[0]        = 0.5f;
                        sLfo.fArg[1]        = 0.0f;
                        break;
                    case meta::phaser::OSC_LAST:
                        sLfo.fArg[0]        = 0.5f;
                        sLfo.fArg[1]        = 0.5f;
                        break;
                    case meta::phaser::OSC_FULL:
                    default:
                        sLfo.fArg[0]        = 1.0f;
                        sLfo.fArg[1]        = 0.0f;
                        break;
                }

                // Update LFO image
                float k                 = sLfo.fArg[0] / (meta::phaser::LFO_MESH_SIZE - 1);
                for (size_t j=0; j<meta::phaser::LFO_MESH_SIZE; ++j)
                    sLfo.vLfoMesh[j]        = sLfo.pFunc(j * k + sLfo.fArg[1]);
            }

            sLfo.nInitPhase         = phase_to_int(sLfo.pInitPhase->value());

            // Update channels
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];

                // For Mid/Side switch change, clear the buffers
                if (mid_side != bMS)
                    c->sFeedback.clear();

                // Update bypass
                c->sBypass.set_bypass(bypass);

                // Setup hi-pass filter for processed signal
                dspu::filter_params_t fp;
                size_t hp_slope = pHpfMode->value() * 2;
                fp.nType        = (hp_slope > 0) ? dspu::FLT_BT_BWC_HIPASS : dspu::FLT_NONE;
                fp.fFreq        = pHpfFreq->value();
                fp.fFreq2       = fp.fFreq;
                fp.fGain        = 1.0f;
                fp.nSlope       = hp_slope;
                fp.fQuality     = 0.0f;
                c->sEq.set_params(0, &fp);

                // Setup low-pass filter for processed signal
                size_t lp_slope = pLpfMode->value() * 2;
                fp.nType        = (lp_slope > 0) ? dspu::FLT_BT_BWC_LOPASS : dspu::FLT_NONE;
                fp.fFreq        = pLpfFreq->value();
                fp.fFreq2       = fp.fFreq;
                fp.fGain        = 1.0f;
                fp.nSlope       = lp_slope;
                fp.fQuality     = 0.0f;
                c->sEq.set_params(1, &fp);
            }

            bMS                     = mid_side;
            bMono                   = (pMono != NULL) ? pMono->value() >= 0.5f : false;
        }

        void phaser::process(size_t samples)
        {
            // Reset phase if phase request is pending
            if (sReset.pending())
            {
                sLfo.nPhase             = 0;
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];
                    c->sFeedback.clear();
                }
                sReset.commit();
            }

            // Perform the routing
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->vIn                  = c->pIn->buffer<float>();
                c->vOut                 = c->pOut->buffer<float>();

                // Measure the input level
                c->pInLevel->set_value(dsp::abs_max(c->vIn, samples) * fInGain);
            }

            for (size_t offset=0; offset<samples; )
            {
                const uint32_t to_do    = lsp_min(samples - offset, BUFFER_SIZE);
                const float k_to_do     = 1.0f / float(to_do);

                // Convert to Mid/Side if needed
                if ((bMS) && (nChannels > 1))
                {
                    dsp::lr_to_ms(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vIn,
                        vChannels[1].vIn,
                        to_do);

                    if (fOldInGain != fInGain)
                    {
                        dsp::lramp1(vChannels[0].vBuffer, fOldInGain, fInGain, to_do);
                        dsp::lramp1(vChannels[1].vBuffer, fOldInGain, fInGain, to_do);
                    }
                }
                else
                {
                    dsp::lramp2(vChannels[0].vBuffer, vChannels[0].vIn, fOldInGain, fInGain, to_do);
                    if (nChannels > 1)
                        dsp::lramp2(vChannels[1].vBuffer, vChannels[1].vIn, fOldInGain, fInGain, to_do);
                }

                // Do audio processing
                uint32_t phase          = sLfo.nPhase;

                for (size_t nc = 0; nc < nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];
                    phase                   = sLfo.nPhase;

                    // Store original buffer contents for mixing with processed data
                    dsp::copy(vBuffer, c->vBuffer, to_do);

                    // Process each sample in the buffer
                    for (size_t i=0; i<to_do; ++i)
                    {
                        const float s           = i * k_to_do;
//                        const float fmin        = elerp(sLfo.fOldMinFreq, sLfo.fMinFreq, s); // Minimum frequency
//                        const float fmax        = elerp(sLfo.fOldMaxFreq, sLfo.fMaxFreq, s); // Maximum frequency

//                        // Apply each all-pass filter to the signal
//                        for (size_t j=0; j<nFilters; ++j)
//                        {
//                            filter_t *f             = &c->vFilters[j];
//                            uint32_t i_phase        = (phase + ilerp(sLfo.nOldInitPhase + f->nPhase, sLfo.nInitPhase + f->nPhase, s)) & PHASE_MASK;
//
//                            // TODO: apply filters here
//                        }

                        // Update current phase
                        phase                   = (phase + ilerp(sLfo.nOldPhaseStep, sLfo.nPhaseStep, s)) & PHASE_MASK;
                    }

                    // TODO: process feedback

                    // Apply output equalizer
                    c->sEq.process(c->vBuffer, c->vBuffer, to_do);
                }

                // Update LFO parameters
                sLfo.nPhase             = phase;
                sLfo.nOldInitPhase      = sLfo.nInitPhase;
                sLfo.nOldPhaseStep      = sLfo.nPhaseStep;

                // Convert back to left-right if needed
                if ((bMS) && (nChannels > 1))
                {
                    dsp::ms_to_lr(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        to_do);
                }

                // Apply Dry/Wet and measure output level
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

                    // Mix dry/wet
                    dsp::lramp2(vBuffer, c->vBuffer, fOldWetGain, fWetGain, to_do);
                    dsp::lramp_add3(c->vBuffer, vBuffer, c->vBuffer, fOldDryGain*fOldInGain, fDryGain*fInGain, to_do);
                    c->pOutLevel->set_value(dsp::abs_max(c->vBuffer, to_do));
                }

                // Apply mono compatibility switch
                if ((nChannels > 1) && (bMono))
                {
                    dsp::lr_to_mid(vChannels[0].vBuffer, vChannels[0].vBuffer, vChannels[1].vBuffer, to_do);
                    dsp::copy(vChannels[1].vBuffer, vChannels[0].vBuffer, to_do);
                }

                // Apply bypass and update buffer pointers
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

                    // Apply bypass
                    c->sBypass.process(c->vOut, c->vIn, c->vBuffer, to_do);

                    // Move pointers
                    c->vIn                 += to_do;
                    c->vOut                += to_do;
                }

                // Commit values

//                fOldFeedGain        = fFeedGain;
//                nOldFeedDelay       = nFeedDelay;
                fOldInGain          = fInGain;
                fOldDryGain         = fDryGain;
                fOldWetGain         = fWetGain;

                offset             += to_do;
            }

            // Synchronize LFO state
            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                // Output filter meters
                for (size_t j=0; j < nFilters; ++j)
                {
                    filter_t *f         = &c->vFilters[j];

                    f->pPhase->set_value(f->fOutPhase * 360.0f);
                    f->pShift->set_value(f->fOutShift);
                    f->pOutFreq->set_value(f->fOutFreq);
                }

                // Clear other filter meters
                for (size_t j=0; j < meta::phaser::FILTERS_MAX; ++j)
                {
                    filter_t *f         = &c->vFilters[j];

                    f->pPhase->set_value(0.0f);
                    f->pShift->set_value(0.0f);
                    f->pOutFreq->set_value(meta::phaser::LFO_FREQ_MIN);
                }
            }

            // Need to synchronize LFO mesh?
            if (sLfo.bSyncMesh)
            {
                plug::mesh_t *mesh      = (sLfo.pMesh != NULL) ? sLfo.pMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    dsp::copy(mesh->pvData[0], vLfoPhase, meta::phaser::LFO_MESH_SIZE);

                    for (size_t j=0; j<nFilters; ++j)
                    {
                        const filter_t *f       = &vChannels[0].vFilters[j];
                        dsp::mul_k3(mesh->pvData[j+1], sLfo.vLfoMesh, f->fNormScale, meta::phaser::LFO_MESH_SIZE);
                        dsp::add_k2(mesh->pvData[j+1], f->fNormShift, meta::phaser::LFO_MESH_SIZE);
                    }

                    mesh->data(nFilters + 1, meta::phaser::LFO_MESH_SIZE);

                    sLfo.bSyncMesh      = false;
                }
            }

            // Request the inline display for redraw
            if (pWrapper != NULL)
                pWrapper->query_display_draw();
        }

        void phaser::ui_activated()
        {
            sLfo.bSyncMesh          = true;
        }

        void phaser::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            // TODO

            v->write("pData", pData);
        }

    } /* namespace plugins */
} /* namespace lsp */


