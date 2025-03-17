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

#ifndef PRIVATE_PLUGINS_PHASER_H_
#define PRIVATE_PLUGINS_PHASER_H_

#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/ctl/Toggle.h>
#include <lsp-plug.in/dsp-units/filters/Equalizer.h>
#include <lsp-plug.in/dsp-units/misc/lfo.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/plug-fw/plug.h>
#include <private/meta/phaser.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Base class for the latency compensation delay
         */
        class phaser: public plug::Module
        {
            protected:
                typedef struct filter_t
                {
                    uint32_t                nPhase;             // Phase shift relative to global LFO
                    float                   fNormShift;         // Normalized shift
                    float                   fNormScale;         // Normalized scale
                    float                   fOutPhase;          // Output phase value
                    float                   fOutShift;          // Output shift value
                    float                   fOutFreq;           // Output frequency

                    plug::IPort            *pPhase;             // Output phase
                    plug::IPort            *pShift;             // Output delay shift
                    plug::IPort            *pOutFreq;           // Actual frequency
                } filter_t;

                typedef struct lfo_t
                {
                    uint32_t                nType;              // LFO type
                    uint32_t                nPeriod;            // LFO period (full, first half, second half)
                    float                   fOverlap;           // LFO overlapping
                    float                   fMinFreq;           // Minimum frequency
                    float                   fMaxFreq;           // Maximum frequency
                    uint32_t                nOldInitPhase;      // Old init phase
                    uint32_t                nInitPhase;         // Initial phase
                    float                   fIFilterPhase;      // Inter-filter phase
                    float                   fIChanPhase;        // Inter-channel phase
                    float                   fArg[2];            // LFO arguments

                    dspu::lfo::function_t   pFunc;              // LFO function
                    float                  *vLfoMesh;           // LFO mesh amplitude data

                    bool                    bSyncMesh;          // Need to synchronize mesh with UI

                    plug::IPort            *pType;              // LFO type
                    plug::IPort            *pPeriod;            // LFO period
                    plug::IPort            *pOverlap;           // Overlap
                    plug::IPort            *pMinFreq;           // Minimum frequency
                    plug::IPort            *pMaxFreq;           // Maximum frequency
                    plug::IPort            *pInitPhase;         // Initial phase
                    plug::IPort            *pIFilterPhase;      // Inter-filter phase
                    plug::IPort            *pIChannelPhase;     // Inter-channel phase
                    plug::IPort            *pMesh;              // Mesh data
                } lfo_t;

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass            sBypass;            // Bypass
                    dspu::Equalizer         sEq;                // Equalizer for processed signal
                    filter_t                vFilters[meta::phaser::FILTERS_MAX];    // Filters

                    // Parameters
                    float                   *vIn;               // Input buffer
                    float                   *vOut;              // Output buffer
                    float                   *vBuffer;           // Processed signal

                    // Data ports ports
                    plug::IPort            *pIn;                // Input port
                    plug::IPort            *pOut;               // Output port
                    plug::IPort            *pInLevel;           // Input level meter
                    plug::IPort            *pOutLevel;          // Output level meter
                } channel_t;

            protected:
                static dspu::lfo::function_t    all_lfo_functions[];

            protected:
                uint32_t                nChannels;          // Number of channels
                uint32_t                nFilters;           // Number of active filters

                dspu::Toggle            sReset;             // Reset toggle
                channel_t              *vChannels;          // Number of channels
                lfo_t                   sLfo;               // Low-frequency oscillator
                float                  *vBuffer;            // Temporary buffer for processing
                float                  *vLfoPhase;          // Buffer that stores LFO phase

                plug::IPort            *pBypass;            // Bypass switch
                plug::IPort            *pMono;              // Mono compatibility test
                plug::IPort            *pMS;                // Mid/Side switch
                plug::IPort            *pInvPhase;          // Phase inverse
                plug::IPort            *pHpfMode;           // High-pass filter mode
                plug::IPort            *pHpfFreq;           // High-pass filter frequency
                plug::IPort            *pLpfMode;           // Low-pass filter mode
                plug::IPort            *pLpfFreq;           // Low-pass filter frequency

                plug::IPort            *pRate;              // Rate
                plug::IPort            *pFraction;          // Time fraction
                plug::IPort            *pTempo;             // Tempo
                plug::IPort            *pTempoSync;         // Tempo sync
                plug::IPort            *pTimeMode;          // Time computing method
                plug::IPort            *pReset;             // Reset phase to initial value

                plug::IPort            *pFilters;           // Number of filters used
                plug::IPort            *pCrossfade;         // Crossfade length
                plug::IPort            *pCrossfadeType;     // Crossfade type

                plug::IPort            *pFeedOn;            // Enable feedback
                plug::IPort            *pFeedGain;          // Feedback gain
                plug::IPort            *pFeedDelay;         // Feedback delay
                plug::IPort            *pFeedPhase;         // Feedback phase

                plug::IPort            *pInGain;            // Input gain
                plug::IPort            *pDryGain;           // Dry gain
                plug::IPort            *pWetGain;           // Wet gain
                plug::IPort            *pDryWet;            // Dry/wet balance
                plug::IPort            *pOutGain;           // Output gain

                uint8_t                *pData;              // Allocated data

            protected:
                void                do_destroy();

            public:
                explicit phaser(const meta::plugin_t *meta);
                phaser(const phaser &) = delete;
                phaser(phaser &&) = delete;
                virtual ~phaser() override;

                phaser & operator = (const phaser &) = delete;
                phaser & operator = (phaser &&) = delete;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        update_sample_rate(long sr) override;
                virtual void        update_settings() override;
                virtual void        process(size_t samples) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_PHASER_H_ */

