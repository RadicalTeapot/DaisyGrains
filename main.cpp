#include "daisysp.h"
#include "daisy_seed.h"
#include <cmath>
#include "Grain.h"

using namespace daisysp;
using namespace daisy;

static DaisySeed hw;
#ifdef DEBUG
static CpuLoadMeter meter;
#endif
static CrossFade dryWetControl, reverbDryWetControl;
static ReverbSc reverb;

const int   kLoopDelayLength    = 25;                       // In ms
const int   kAdcReadInterval    = 25;                       // In ms
const int   kCPUReportInterval  = 500;                      // In ms
const float kOnePoleCvCoeff     = 0.5f;                     // time = 1 / (sampleRate * coeff) so ~0.05s at 40Hz (1/kDelayLoopLength)
const int   kBufferDuration     = 10;                       // In seconds
const int   kBufferSize         = 48000 * kBufferDuration;  // Assume 48kHz sample rate

const int kGrainCount = 5;
static graindelay::Grain grains[kGrainCount];
float DSY_SDRAM_BSS grainBuffer[kBufferSize * kGrainCount];

// TODO Test if feedback works as expected

// Use only 5v tolerant I/O pins (see datasheet)
enum AdcChannels {
    AdcMixIn = 0,       // Pin A0
    AdcGrainChanceIn,   // Pin A1
    AdcGrainSizeIn,     // Pin A4
    AdcReverbMixIn,     // Pin A5
    AdcFeedbackIn,      // Pin A9
    ADC_CHANNEL_COUNT,
};

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
#ifdef DEBUG
    meter.OnBlockStart();
#endif

    float voicesMix[2], grainProcess, inValue, reverbOut[2], reverbMix[2];
    int j;
    for(size_t i = 0; i < size; i += 2)
    {
        inValue = in[i];

        voicesMix[0] = 0;
        voicesMix[1] = 0;
        for (j = 0; j < kGrainCount; j++)
        {
            grainProcess = grains[j].Process(inValue);
            voicesMix[0] += grainProcess * (grains[j].GetPan() * -0.5 + 0.5);
            voicesMix[1] += grainProcess * (grains[j].GetPan() * 0.5 + 0.5);
        }

        // TODO Make processing the reverb conditional to reverb mix being > 0
        reverb.Process(voicesMix[0], voicesMix[1], &reverbOut[0], &reverbOut[1]);
        for (j = 0; j < 2; j++)
        {
            reverbMix[j] = reverbDryWetControl.Process(voicesMix[j], reverbOut[j]);
            out[i+j] = dryWetControl.Process(inValue, reverbMix[j]);
        }
    }

#ifdef DEBUG
    meter.OnBlockEnd();
#endif
}

void InitGrains(const float sampleRate)
{
    const float readSpeeds[kGrainCount] = {0.5f, 0.25f, 1.4983f, -1.0f, -0.5f};    // In samples
    const float mixValues[kGrainCount] = {0.25f, 0.2f, 0.1f, 0.3f, 0.15f};         // Sum to 1

    for (int i = 0; i < kGrainCount; i++) {
        grains[i].Init(sampleRate, &grainBuffer[kBufferSize * i], kBufferSize);
        grains[i].SetAmp(mixValues[i] * 1.66f);
        grains[i].SetSpeed(readSpeeds[i]);
    }
}

void AdcInit()
{
    // Configure ADC pins
    AdcChannelConfig adcChannelConfig[ADC_CHANNEL_COUNT];
    adcChannelConfig[AdcMixIn].InitSingle(daisy::seed::A0);
    adcChannelConfig[AdcGrainChanceIn].InitSingle(daisy::seed::A1);
    adcChannelConfig[AdcGrainSizeIn].InitSingle(daisy::seed::A4);
    adcChannelConfig[AdcReverbMixIn].InitSingle(daisy::seed::A5);
    adcChannelConfig[AdcFeedbackIn].InitSingle(daisy::seed::A9);
    hw.adc.Init(adcChannelConfig, ADC_CHANNEL_COUNT);
}

void ReadAdcIn(float &dryWet, float &reverbDryWet, float &grainDensity, float &grainSize, float &feedback)
{
    fonepole(dryWet, hw.adc.GetFloat(AdcMixIn), kOnePoleCvCoeff);
    fonepole(reverbDryWet, hw.adc.GetFloat(AdcReverbMixIn), kOnePoleCvCoeff);
    fonepole(grainDensity, hw.adc.GetFloat(AdcGrainChanceIn), kOnePoleCvCoeff);
    fonepole(grainSize, hw.adc.GetFloat(AdcGrainSizeIn), kOnePoleCvCoeff);
    fonepole(feedback, hw.adc.GetFloat(AdcFeedbackIn), kOnePoleCvCoeff);
}

void Init()
{
    // Initialize seed hardware
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(256);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    AdcInit();

    // Configure LFOs and envelopes
    float sampleRate = hw.AudioSampleRate();
    InitGrains(sampleRate);

#ifdef DEBUG
    // initialize the load meter so that it knows what time is available for the processing
    meter.Init(sampleRate, hw.AudioBlockSize());

    hw.StartLog();
#endif

    // Configure dry/wet control
    dryWetControl.Init(CROSSFADE_CPOW);
    reverbDryWetControl.Init(CROSSFADE_CPOW);

    // Configure reverb
    reverb.Init(sampleRate);
    reverb.SetFeedback(0.9);

    // Start ADC
    hw.adc.Start();

    // Start callback
    hw.StartAudio(AudioCallback);
}

void Run()
{
    float dryWet = 0, reverbDryWet = 0, grainDensity = 0, grainSize = 0, feedback = 0;
    int timeUntilNextAdcRead = 0;
#ifdef DEBUG
    int timeUntilNextCPUReport = 0;
#endif
	for (;;)
	{
        if (timeUntilNextAdcRead <= 0)
        {
            ReadAdcIn(dryWet, reverbDryWet, grainDensity, grainSize, feedback);
            dryWetControl.SetPos(dryWet);
            reverbDryWetControl.SetPos(reverbDryWet);
            for (int i = 0; i < kGrainCount; i++)
            {
                grains[i].SetGrainDensity(grainDensity);
                grains[i].SetDuration(grainSize);
                grains[i].SetFeedback(feedback);
            }
            timeUntilNextAdcRead = kAdcReadInterval;
        }
        timeUntilNextAdcRead -= kLoopDelayLength;

#ifdef DEBUG
        if (timeUntilNextCPUReport <= 0)
        {
            // get the current load (smoothed value and peak values)
            const float avgLoad = meter.GetAvgCpuLoad();
            const float maxLoad = meter.GetMaxCpuLoad();
            const float minLoad = meter.GetMinCpuLoad();
            // print it to the serial connection (as percentages)
            hw.PrintLine("Processing Load %:");
            hw.PrintLine("Max: " FLT_FMT3, FLT_VAR3(maxLoad * 100.0f));
            hw.PrintLine("Avg: " FLT_FMT3, FLT_VAR3(avgLoad * 100.0f));
            hw.PrintLine("Min: " FLT_FMT3, FLT_VAR3(minLoad * 100.0f));

            timeUntilNextCPUReport = kCPUReportInterval;
        }
        timeUntilNextCPUReport -= kLoopDelayLength;
#endif

        System::Delay(kLoopDelayLength);
	}
}

int main(void)
{
    Init();
    Run();
}
