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
#include <lsp-plug.in/dsp-units/misc/quickmath.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>
#include <lsp-plug.in/shared/id_colors.h>

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

            sLfo.bSyncMesh      = true;

            sLfo.pOverlap       = NULL;
            sLfo.pMinFreq       = NULL;
            sLfo.pMaxFreq       = NULL;
            sLfo.pInitPhase     = NULL;
            sLfo.pIFilterPhase  = NULL;
            sLfo.pIChannelPhase = NULL;
            sLfo.pMesh          = NULL;

            vBuffer             = NULL;
            vLfoPhase           = NULL;

            fRevSampleRate      = 0.0f;
            fRevQuality         = 0.5f;
            fRate               = 0.0f;
            fOldDepth           = GAIN_AMP_0_DB;
            fDepth              = GAIN_AMP_0_DB;
            nCrossfade          = 0;
            fCrossfade          = 0.0f;
            fRevCrossfade       = 0.0f;
            fOldInGain          = GAIN_AMP_0_DB;
            fInGain             = fOldInGain;
            fOldDryGain         = GAIN_AMP_M_INF_DB;
            fDryGain            = fOldDryGain;
            fOldWetGain         = GAIN_AMP_0_DB;
            fWetGain            = fOldWetGain;
            fOldFeedGain        = GAIN_AMP_M_INF_DB;
            fFeedGain           = GAIN_AMP_M_INF_DB;
            fOldFeedDelay       = 0.0f;
            fFeedDelay          = 0.0f;

            bMS                 = false;
            bMono               = false;
            bCustomLfo          = false;
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
            pDepth              = NULL;
            pFraction           = NULL;
            pTempo              = NULL;
            pTempoSync          = NULL;
            pTimeMode           = NULL;
            pReset              = NULL;

            pFilters            = NULL;
            pFilterQuality      = NULL;
            pCrossfade          = NULL;

            pFeedOn             = NULL;
            pFeedGain           = NULL;
            pFeedDelay          = NULL;
            pFeedPhase          = NULL;

            pInGain             = NULL;
            pDryGain            = NULL;
            pWetGain            = NULL;
            pDryWet             = NULL;
            pOutGain            = NULL;

            pIDisplay           = NULL;

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
                buf_sz * 4 +            // vBuffer
                mesh_buf_sz +           // vLfoPhase
                mesh_buf_sz * 2 +       // channel_t::vLfoMesh
                nChannels * buf_sz;     // channel_t::vBuffer

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, to_alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;
            lsp_guard_assert(uint8_t *save   = ptr);

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);
            vBuffer                 = advance_ptr_bytes<float>(ptr, buf_sz * 4);
            vLfoPhase               = advance_ptr_bytes<float>(ptr, mesh_buf_sz);

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

                    f->sAllpass[0]          = 0.0f;
                    f->sAllpass[1]          = 0.0f;
                    f->sAllpass[2]          = 0.0f;
                    f->sAllpass[3]          = 0.0f;

                    f->nPhase               = 0;
                    f->nActPhase            = 0;
                    f->fNormShift           = 0.0f;
                    f->fNormScale           = 0.0f;
                    f->fOutPhase            = 0.0f;
                    f->fOutShift            = 0.0f;
                    f->fOutFreq             = 0.0f;

                    f->pPhase               = NULL;
                    f->pShift               = NULL;
                    f->pOutFreq             = NULL;
                }

                c->nLfoType             = -1;
                c->nLfoPeriod           = -1;
                c->vLfoArg[0]           = 0;
                c->vLfoArg[1]           = 0;
                c->pLfoFunc             = NULL;
                c->vLfoMesh             = advance_ptr_bytes<float>(ptr, mesh_buf_sz);

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vBuffer              = advance_ptr_bytes<float>(ptr, buf_sz);

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pInLevel             = NULL;
                c->pOutLevel            = NULL;
                c->pLfoType             = NULL;
                c->pLfoPeriod           = NULL;
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
            BIND_PORT(pDepth);
            BIND_PORT(pFraction);
            SKIP_PORT("Denominator");   // Skip denominator
            BIND_PORT(pTempo);
            BIND_PORT(pTempoSync);
            BIND_PORT(pTimeMode);
            BIND_PORT(pReset);

            // LFO settings
            lsp_trace("Binding LFO settings");
            BIND_PORT(pFilters);
            BIND_PORT(pFilterQuality);
            BIND_PORT(pCrossfade);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c = &vChannels[i];
                BIND_PORT(c->pLfoType);
                BIND_PORT(c->pLfoPeriod);
            }

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

            // Destroy inline display buffer
            if (pIDisplay != NULL)
            {
                pIDisplay->destroy();
                pIDisplay   = NULL;
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

        inline float phaser::process_allpass(float *d, float freq, float s)
        {
            // Update biquad filter
            const float cc      = dspu::quick_cosf(fRevSampleRate * freq);
            const float alpha   = sqrtf(1.0f - cc*cc) * fRevQuality;

            const float k_b0    = 1.0f / (1.0f + alpha);
            const float b0      = (1.0f - alpha) * k_b0;
            const float b1      = 2.0f * cc * k_b0;

            // Apply processing
            const float s2      = b0 * s + d[0];

            // Shift buffer
            d[0]                = d[1] + b1 * (s2 - s);
            d[1]                = s - b0 * s2;

            return s2;
        }

        void phaser::update_sample_rate(long sr)
        {
            plug::Module::update_sample_rate(sr);

            // Update sample rate for the bypass processors
            fRevSampleRate      = 2.0f * M_PI / float(fSampleRate);
            size_t max_feedback = dspu::millis_to_samples(sr, meta::phaser::FEEDBACK_DELAY_MAX);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                c->sBypass.init(sr);
                c->sFeedback.init(max_feedback + BUFFER_SIZE*2);
                c->sEq.set_sample_rate(sr);

                // Reset all-pass filters memory
                for (size_t j=0; j<meta::phaser::FILTERS_MAX; ++j)
                {
                    filter_t *f             = &c->vFilters[j];

                    f->sAllpass[0]          = 0.0f;
                    f->sAllpass[1]          = 0.0f;
                    f->sAllpass[2]          = 0.0f;
                    f->sAllpass[3]          = 0.0f;
                }
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
            fOldDepth               = fDepth;
            fDepth                  = pDepth->value();

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

            // Update state of the 'reset' trigger
            sReset.submit(pReset->value());

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
            fOldFeedDelay           = fFeedDelay;
            fFeedDelay              = dspu::millis_to_samples(fSampleRate, pFeedDelay->value());
            fOldFeedGain            = fFeedGain;
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            nCrossfade              = float(PHASE_MAX) * crossfade * 2.0f;
            fCrossfade              = PHASE_COEFF * (1.0f - crossfade);
            fRevCrossfade           = (fCrossfade > 0.0f) ? 1.0f / float(nCrossfade) : 0.0f;

            // LFO setup
            const size_t filters    = lsp_min(pFilters->value() + 1, meta::phaser::FILTERS_MAX);
            const float if_phase    = sLfo.pIFilterPhase->value();
            const float ichan_phase = (sLfo.pIChannelPhase != NULL) ? sLfo.pIChannelPhase->value() : 0.0f;
            const float overlap     = sLfo.pOverlap->value() * 0.01f;

            fRevQuality             = 0.5f / pFilterQuality->value();

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
                const float ovl_width   = dspu::lerp(1.0f / nFilters, 1.0f, sLfo.fOverlap);
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
            bool custom_lfo         = false;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Update LFO preferences
                size_t lfo_type         = size_t(c->pLfoType->value());
                size_t lfo_period       = size_t(c->pLfoPeriod->value());
                if (i > 0)
                {
                    custom_lfo              = lfo_type > 0;
                    if (!custom_lfo)
                        lfo_period              = vChannels[0].nLfoPeriod;
                    lfo_type                = (custom_lfo) ? lfo_type - 1 : vChannels[0].nLfoType;
                }

                // The form of the LFO has changed?
                if ((lfo_type != c->nLfoType) || (lfo_period != c->nLfoPeriod))
                {
                    c->nLfoType             = lfo_type;
                    c->nLfoPeriod           = lfo_period;
                    c->pLfoFunc             = all_lfo_functions[lfo_type];
                    sLfo.bSyncMesh          = true;

                    // Select the function coefficients
                    switch (lfo_period)
                    {
                        case meta::phaser::OSC_FIRST:
                            c->vLfoArg[0]       = 0.5f;
                            c->vLfoArg[1]       = 0.0f;
                            break;
                        case meta::phaser::OSC_LAST:
                            c->vLfoArg[0]       = 0.5f;
                            c->vLfoArg[1]       = 0.5f;
                            break;
                        case meta::phaser::OSC_FULL:
                        default:
                            c->vLfoArg[0]       = 1.0f;
                            c->vLfoArg[1]       = 0.0f;
                            break;
                    }

                    // Update LFO image
                    float k                 = c->vLfoArg[0] / (meta::phaser::LFO_MESH_SIZE - 1);
                    for (size_t j=0; j<meta::phaser::LFO_MESH_SIZE; ++j)
                        c->vLfoMesh[j]          = c->pLfoFunc(j * k + c->vLfoArg[1]);
                }
            }

            const float freq_limit  = fSampleRate * 0.49f;
            sLfo.nInitPhase         = phase_to_int(sLfo.pInitPhase->value());
            sLfo.fOldMinFreq        = lsp_min(sLfo.fOldMinFreq, freq_limit);
            sLfo.fMinFreq           = lsp_min(sLfo.pMinFreq->value(), freq_limit);
            sLfo.fOldMaxFreq        = lsp_min(sLfo.fOldMaxFreq, freq_limit);
            sLfo.fMaxFreq           = lsp_min(sLfo.pMaxFreq->value(), freq_limit);

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
            bCustomLfo              = custom_lfo;
            bMono                   = (pMono != NULL) ? pMono->value() >= 0.5f : false;
        }

        inline void phaser::lerp_frequencies(float *dst, float min, float max, size_t count)
        {
            if (min != max)
            {
                const float k = 1.0f / float(count);
                for (size_t i=0; i<count; ++i)
                    dst[i]                  = dspu::quick_elerp(min, max, i*k);
            }
            else
                dsp::fill(dst, min, count);
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

                // Compute additional parameters
                float *fmin             = &vBuffer[0];
                float *fmax             = &fmin[to_do];
                float *depth            = &fmax[to_do];

                lerp_frequencies(fmin, sLfo.fOldMinFreq, sLfo.fMinFreq, to_do);
                lerp_frequencies(fmax, sLfo.fOldMaxFreq, sLfo.fMaxFreq, to_do);
                dsp::lramp_set1(depth, fOldDepth, fDepth, to_do);

                for (size_t nc = 0; nc < nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];
                    phase                   = sLfo.nPhase;

                    // Process each sample in the buffer
                    for (size_t i=0; i<to_do; ++i)
                    {
                        const float s           = i * k_to_do;
                        float sample            = c->vBuffer[i];
                        const uint32_t lfo_phase= dspu::ilerp(sLfo.nOldInitPhase, sLfo.nInitPhase, s);

                        // Apply feedback
                        const float fb_delay    = dspu::lerp(fOldFeedDelay, fFeedDelay, s);
                        const float fb_sample   = c->sFeedback.get(fb_delay);
                        sample                 += fb_sample * dspu::lerp(fOldFeedGain, fFeedGain, s);

                        // Apply each all-pass filter to the signal
                        for (size_t j=0; j<nFilters; ++j)
                        {
                            filter_t *f             = &c->vFilters[j];
                            uint32_t i_phase        = (phase + lfo_phase + f->nPhase) & PHASE_MASK;
                            const float o_phase     = i_phase * fCrossfade;
                            float c_phase           = o_phase * c->vLfoArg[0] + c->vLfoArg[1];
                            float c_func            = f->fNormScale * c->pLfoFunc(c_phase) + f->fNormShift;
                            float c_freq            = dspu::quick_elerp(fmin[i], fmax[i], c_func);

                            // Check if LFO went to the next round
                            if ((nCrossfade > 0) && (f->nActPhase > i_phase))
                            {
                                f->sAllpass[2]          = f->sAllpass[0];
                                f->sAllpass[3]          = f->sAllpass[1];
                                f->sAllpass[0]          = 0.0f;
                                f->sAllpass[1]          = 0.0f;
                            }
                            f->nActPhase            = i_phase;

                            // Perform cross-fade
                            if (i_phase < nCrossfade)
                            {
                                const float new_sample  = process_allpass(&f->sAllpass[0], c_freq, sample);
                                const float mix         = float(i_phase) * fRevCrossfade;
                                i_phase                 = i_phase + PHASE_MAX;
                                c_phase                 = i_phase * fCrossfade * c->vLfoArg[0] + c->vLfoArg[1];
                                c_func                  = f->fNormScale * c->pLfoFunc(c_phase) + f->fNormShift;
                                c_freq                  = dspu::quick_elerp(fmin[i], fmax[i], c_func);

                                const float old_sample  = process_allpass(&f->sAllpass[2], c_freq, sample);

                                sample                  = dspu::lerp(old_sample, new_sample, mix);
                            }
                            else
                                sample                  = process_allpass(&f->sAllpass[0], c_freq, sample);
                        }

                        // Append sample to feedback
                        c->sFeedback.append(sample);

                        // Add processed sample to the original one
                        c->vBuffer[i]           = c->vBuffer[i] + sample * depth[i];

                        // Update current phase
                        phase                   = (phase + dspu::ilerp(sLfo.nOldPhaseStep, sLfo.nPhaseStep, s)) & PHASE_MASK;
                    }

                    // Sync state of filters
                    {
                        for (size_t j=0; j<nFilters; ++j)
                        {
                            filter_t *f             = &c->vFilters[j];
                            uint32_t i_phase        = (phase + sLfo.nInitPhase + f->nPhase) & PHASE_MASK;
                            const float o_phase     = i_phase * fCrossfade;
                            const float c_phase     = o_phase * c->vLfoArg[0] + c->vLfoArg[1];
                            const float c_func      = f->fNormScale * c->pLfoFunc(c_phase) + f->fNormShift;
                            const float c_freq      = dspu::quick_elerp(sLfo.fMinFreq, sLfo.fMaxFreq, c_func);

                            f->fOutPhase            = o_phase;
                            f->fOutShift            = c_func;
                            f->fOutFreq             = c_freq;
                        }
                    }

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
                    dsp::lramp_add3(c->vBuffer, vBuffer, c->vIn, fOldDryGain*fOldInGain, fDryGain*fInGain, to_do);
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
                fOldFeedDelay       = fFeedDelay;
                fOldFeedGain        = fFeedGain;
                fOldDepth           = fDepth;
                sLfo.fOldMinFreq    = sLfo.fMinFreq;
                sLfo.fOldMaxFreq    = sLfo.fMaxFreq;
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
                for (size_t j=nFilters; j < meta::phaser::FILTERS_MAX; ++j)
                {
                    filter_t *f         = &c->vFilters[j];

                    f->pPhase->set_value(0.0f);
                    f->pShift->set_value(0.0f);
                    f->pOutFreq->set_value(meta::phaser::LFO_FREQ_MIN);
                }

//            #ifdef LSP_TRACE
//                for (size_t j=0; j < meta::phaser::FILTERS_MAX; ++j)
//                {
//                    filter_t *f         = &c->vFilters[j];
//
//                    lsp_trace("channel %d filter %d = {%s=%f, %s=%f, %s=%f}",
//                        int(i), int(j),
//                        f->pPhase->id(),
//                        f->pPhase->value(),
//                        f->pShift->id(),
//                        f->pShift->value(),
//                        f->pOutFreq->id(),
//                        f->pOutFreq->value());
//                }
//            #endif /* LSP_TRACE */
            }

            // Need to synchronize LFO mesh?
            if (sLfo.bSyncMesh)
            {
                plug::mesh_t *mesh      = (sLfo.pMesh != NULL) ? sLfo.pMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    size_t index        = 0;
                    dsp::copy(mesh->pvData[index++], vLfoPhase, meta::phaser::LFO_MESH_SIZE);

                    const size_t lines      = (bCustomLfo) ? 2 : 1;
                    for (size_t i=0; i<lines; ++i)
                    {
                        const channel_t *c      = &vChannels[i];

                        for (size_t j=0; j<nFilters; ++j)
                        {
                            const filter_t *f       = &vChannels[0].vFilters[j];
                            float *data             = mesh->pvData[index++];

                            dsp::mul_k3(data, c->vLfoMesh, f->fNormScale, meta::phaser::LFO_MESH_SIZE);
                            dsp::add_k2(data, f->fNormShift, meta::phaser::LFO_MESH_SIZE);
                        }
                    }

                    mesh->data(index, meta::phaser::LFO_MESH_SIZE);

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

        bool phaser::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // Check proportions
            if (height > width)
                height  = width;

            // Init canvas
            if (!cv->init(width, height))
                return false;
            width   = cv->width();
            height  = cv->height();

            // Clear background
            bool bypassing = vChannels[0].sBypass.bypassing();
            cv->set_color_rgb((bypassing) ? CV_DISABLED : CV_BACKGROUND);
            cv->paint();

            // Draw horizontal and vertical lines
            cv->set_line_width(1.0);
            cv->set_color_rgb((bypassing) ? CV_SILVER: CV_YELLOW, 0.5f);
            for (size_t i=1; i < 8; ++i)
            {
                float y = (float(i) * (45.0f / 360.0f)) * height;
                float x = float(i) * 0.125f * width;
                cv->line(0, y, width, y);
                cv->line(x, 0, x, height);
            }

            // Reuse display
            size_t count        = lsp_max(width, height);
            pIDisplay           = core::IDBuffer::reuse(pIDisplay, 2, count);
            core::IDBuffer *b   = pIDisplay;
            if (b == NULL)
                return false;

            static const uint32_t c_colors[] = {
                CV_MIDDLE_CHANNEL,
                CV_LEFT_CHANNEL, CV_RIGHT_CHANNEL,
                CV_MIDDLE_CHANNEL, CV_SIDE_CHANNEL
            };

            const uint32_t *colors  = &c_colors[0];
            size_t lines            = 1;
            if ((nChannels > 1) && (bCustomLfo))
            {
                colors  = (bMS) ? &c_colors[3] : &c_colors[1];
                lines   = 2;
            }

            bool aa = cv->set_anti_aliasing(true);
            lsp_finally { cv->set_anti_aliasing(aa); };

            cv->set_line_width(2);

            dsp::lramp_set1(b->v[1], 0.0f, height-1, count);

            for (size_t i=0; i<lines; ++i)
            {
                const channel_t *c  = &vChannels[i];

                for (size_t j=0; j<nFilters; ++j)
                {
                    const filter_t *f   = &c->vFilters[j];

                    for (size_t k=0; k<count; ++k)
                    {
                        const size_t ind    = (k * meta::phaser::LFO_MESH_SIZE) / count;
                        b->v[0][k]          = (c->vLfoMesh[ind] * f->fNormScale + f->fNormShift) * width;
                    }

                    // Draw mesh
                    const uint32_t color    = (bypassing || !(active())) ? CV_SILVER : colors[i];
                    cv->set_color_rgb(color);
                    cv->draw_lines(b->v[0], b->v[1], count);
                }
            }

            // Draw dots with lines
            if (active())
            {
                colors  = (nChannels <= 1) ? &c_colors[0] :
                          (bMS) ? &c_colors[3] : &c_colors[1];
                cv->set_line_width(1);

                // Draw lines first
                for (size_t i=0; i<nChannels; ++i)
                {
                    const channel_t *c  = &vChannels[i];

                    for (size_t j=0; j<lines; ++j)
                    {
                        const filter_t *f   = &c->vFilters[j];
                        const float x       = f->fOutShift * width;

                        cv->set_color_rgb(colors[i]);
                        cv->line(x, 0, x, height);
                    }
                }

                // Draw dots next
                for (size_t i=0; i<nChannels; ++i)
                {
                    const channel_t *c  = &vChannels[i];

                    const uint32_t color = (bypassing) ? CV_SILVER : colors[i];
                    Color c1(color), c2(color);
                    c2.alpha(0.9f);

                    for (size_t j=0; j<nFilters; ++j)
                    {
                        const filter_t *f   = &c->vFilters[j];
                        const float x       = f->fOutShift * width;
                        const float y       = f->fOutPhase * height;

                        cv->radial_gradient(x, y, c1, c2, 12);
                        cv->set_color_rgb(0);
                        cv->circle(x, y, 4);
                        cv->set_color_rgb(color);
                        cv->circle(x, y, 3);
                    }
                }
            }

            return true;
        }

        void phaser::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            v->write("nChannels", nChannels);
            v->write("nFilters", nFilters);

            v->write_object("sReset", &sReset);

            v->begin_array("vChannels", vChannels, nChannels);
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    const channel_t *c = &vChannels[i];

                    v->begin_object(c, sizeof(channel_t));
                    {
                        v->write_object("sBypass", &c->sBypass);
                        v->write_object("sFeedback", &c->sFeedback);
                        v->write_object("sEq", &c->sEq);


                        v->begin_array("vFilters", meta::phaser::FILTERS_MAX);
                        {
                            for (size_t j=0; j<meta::phaser::FILTERS_MAX; ++j)
                            {
                                const filter_t *f = &c->vFilters[j];

                                v->writev("sAllpass", f->sAllpass, 4);
                                v->write("nPhase", f->nPhase);
                                v->write("nActPhase", f->nActPhase);
                                v->write("fNormShift", f->fNormShift);
                                v->write("fNormScale", f->fNormScale);
                                v->write("fOutPhase", f->fOutPhase);
                                v->write("fOutShift", f->fOutShift);
                                v->write("fOutFreq", f->fOutFreq);

                                v->write("pPhase", f->pPhase);
                                v->write("pShift", f->pShift);
                                v->write("pOutFreq", f->pOutFreq);
                            }
                        }
                        v->end_array();

                        v->write("nLfoType", c->nLfoType);
                        v->write("nLfoPeriod", c->nLfoPeriod);
                        v->writev("vLfoArg", c->vLfoArg, 2);
                        v->write("pLfoFunc", c->pLfoFunc);
                        v->write("vLfoMesh", c->vLfoMesh);

                        v->write("vIn", c->vIn);
                        v->write("vOut", c->vOut);
                        v->write("vBuffer", c->vBuffer);

                        v->write("pIn", c->pIn);
                        v->write("pOut", c->pOut);
                        v->write("pInLevel", c->pInLevel);
                        v->write("pOutLevel", c->pOutLevel);
                        v->write("pLfoType", c->pLfoType);
                        v->write("pLfoPeriod", c->pLfoPeriod);
                    }
                    v->end_object();
                }
            }
            v->end_array();

            v->begin_object("sLfo", &sLfo, sizeof(lfo_t));
            v->end_object();

            v->write("vBuffer", vBuffer);
            v->write("vLfoPhase", vLfoPhase);

            v->write("fRevSampleRate", fRevSampleRate);
            v->write("fRevQuality", fRevQuality);
            v->write("fRate", fRate);
            v->write("fOldDepth", fOldDepth);
            v->write("fDepth", fDepth);
            v->write("nCrossfade", nCrossfade);
            v->write("fRevCrossfade", fRevCrossfade);
            v->write("fOldInGain", fOldInGain);
            v->write("fInGain", fInGain);
            v->write("fOldDryGain", fOldDryGain);
            v->write("fDryGain", fDryGain);
            v->write("fOldWetGain", fOldWetGain);
            v->write("fWetGain", fWetGain);
            v->write("fOldFeedGain", fOldFeedGain);
            v->write("fFeedGain", fFeedGain);
            v->write("fOldFeedDelay", fOldFeedDelay);
            v->write("fFeedDelay", fFeedDelay);
            v->write("bMS", bMS);
            v->write("bMono", bMono);
            v->write("bCustomLfo", bCustomLfo);
            v->write("bUpdateFilters", bUpdateFilters);

            v->write("pBypass", pBypass);
            v->write("pMono", pMono);
            v->write("pMS", pMS);
            v->write("pInvPhase", pInvPhase);
            v->write("pHpfMode", pHpfMode);
            v->write("pHpfFreq", pHpfFreq);
            v->write("pLpfMode", pLpfMode);
            v->write("pLpfFreq", pLpfFreq);

            v->write("pRate", pRate);
            v->write("pDepth", pDepth);
            v->write("pFraction", pFraction);
            v->write("pTempo", pTempo);
            v->write("pTempoSync", pTempoSync);
            v->write("pTimeMode", pTimeMode);
            v->write("pReset", pReset);

            v->write("pFilters", pFilters);
            v->write("pFilterQuality", pFilterQuality);
            v->write("pCrossfade", pCrossfade);

            v->write("pFeedOn", pFeedOn);
            v->write("pFeedGain", pFeedGain);
            v->write("pFeedDelay", pFeedDelay);
            v->write("pFeedPhase", pFeedPhase);

            v->write("pInGain", pInGain);
            v->write("pDryGain", pDryGain);
            v->write("pWetGain", pWetGain);
            v->write("pDryWet", pDryWet);
            v->write("pOutGain", pOutGain);

            v->write("pIDisplay", pIDisplay);

            v->write("pData", pData);
        }

    } /* namespace plugins */
} /* namespace lsp */


