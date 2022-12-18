#include "daisysp.h"
#include "daisy_seed.h"
#include <cmath>
#include "Grain.h"

using namespace daisysp;
using namespace daisy;

static DaisySeed hw;
static CpuLoadMeter meter;
static CrossFade dryWetControl, reverbDryWetControl;
static ReverbSc reverb;

const float ONE_POLE_CV_COEFF = 0.05f;              // time = 1 / (sample_rate * coeff) so ~0.5ms at 48kHz
const int BUFFER_DURATION = 10;                     // In seconds
const int BUFFER_SIZE = 48000 * BUFFER_DURATION;    // Assume 48kHz sample rate

const int GRAIN_COUNT = 5;
static graindelay::Grain grains[GRAIN_COUNT];
float DSY_SDRAM_BSS grain_buffer[BUFFER_SIZE * GRAIN_COUNT];

// TODO Fix grain start callback (broken atm) or remove if not easy to fix
// TODO Test if feedback works as expected

enum AdcChannels {
    AdcMixIn = 0,       // Pin A0
    AdcGrainChanceIn,   // Pin A1
    AdcGrainSizeIn,     // Pin A2
    AdcReverbMixIn,     // Pin A3
    AdcFeedbackIn,      // Pin A4
    ADC_CHANNEL_COUNT,
};

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    // meter.OnBlockStart();

    float voicesMix[2], grainProcess, inValue, reverbOut[2], reverbMix[2], outValue;
    int j;
    for(size_t i = 0; i < size; i += 2)
    {
        inValue = in[i];

        voicesMix[0] = 0;
        voicesMix[1] = 0;
        for (j = 0; j < GRAIN_COUNT; j++)
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

    // meter.OnBlockEnd();
}

void InitGrains(const float sample_rate)
{
    const float READ_SPEEDS[GRAIN_COUNT] = {0.5f, 0.25f, 1.4983f, -1.0f, -0.5f};    // In samples
    const float MIX_VALUES[GRAIN_COUNT] = {0.25f, 0.2f, 0.1f, 0.3f, 0.15f};         // Sum to 1

    for (int i = 0; i < GRAIN_COUNT; i++) {
        grains[i].Init(sample_rate, &grain_buffer[BUFFER_SIZE * i], BUFFER_SIZE);
        grains[i].SetAmp(MIX_VALUES[i] * 1.66f);
        grains[i].SetSpeed(READ_SPEEDS[i]);
    }
}

void AdcInit()
{
    // Configure ADC pins
    AdcChannelConfig adcChannelConfig[ADC_CHANNEL_COUNT];
    adcChannelConfig[AdcMixIn].InitSingle(daisy::seed::A0);
    adcChannelConfig[AdcGrainChanceIn].InitSingle(daisy::seed::A1);
    adcChannelConfig[AdcGrainSizeIn].InitSingle(daisy::seed::A2);
    adcChannelConfig[AdcReverbMixIn].InitSingle(daisy::seed::A3);
    adcChannelConfig[AdcFeedbackIn].InitSingle(daisy::seed::A4);
    hw.adc.Init(adcChannelConfig, ADC_CHANNEL_COUNT);
}

void ReadAdcIn(float &dryWet, float &reverbDryWet, float &grainDensity, float &grainSize, float &feedback)
{
    fonepole(dryWet, hw.adc.GetFloat(AdcMixIn), ONE_POLE_CV_COEFF);
    fonepole(reverbDryWet, hw.adc.GetFloat(AdcReverbMixIn), ONE_POLE_CV_COEFF);
    fonepole(grainDensity, hw.adc.GetFloat(AdcGrainChanceIn), ONE_POLE_CV_COEFF);
    fonepole(grainSize, hw.adc.GetFloat(AdcGrainSizeIn), ONE_POLE_CV_COEFF);
    fonepole(feedback, hw.adc.GetFloat(AdcFeedbackIn), ONE_POLE_CV_COEFF);
}

void Init()
{
    // Initialize seed hardware
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(256);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    // initialize the load meter so that it knows what time is available for the processing:
    // meter.Init(sample_rate, hw.AudioBlockSize());
    hw.StartLog();

    AdcInit();

    // Configure LFOs and envelopes
    float sample_rate = hw.AudioSampleRate();
    InitGrains(sample_rate);

    // Configure dry/wet control
    dryWetControl.Init(CROSSFADE_CPOW);
    reverbDryWetControl.Init(CROSSFADE_CPOW);

    // Configure reverb
    reverb.Init(sample_rate);
    reverb.SetFeedback(0.9);

    // Start ADC
    hw.adc.Start();

    // Start callback
    hw.StartAudio(AudioCallback);
}

void Run()
{
    float dryWet = 0, reverbDryWet = 0, grainDensity = 0, grainSize = 0, feedback = 0;
	for (;;)
	{
        ReadAdcIn(dryWet, reverbDryWet, grainDensity, grainSize, feedback);
        dryWetControl.SetPos(dryWet);
        reverbDryWetControl.SetPos(reverbDryWet);
        for (int i = 0; i < GRAIN_COUNT; i++)
        {
            grains[i].SetGrainDensity(grainDensity);
            grains[i].SetDuration(grainSize);
            grains[i].SetFeedback(feedback);
        }

        System::Delay(25);

        // // get the current load (smoothed value and peak values)
        // const float avgLoad = meter.GetAvgCpuLoad();
        // const float maxLoad = meter.GetMaxCpuLoad();
        // const float minLoad = meter.GetMinCpuLoad();
        // // print it to the serial connection (as percentages)
        // hw.PrintLine("Processing Load %:");
        // hw.PrintLine("Max: " FLT_FMT3, FLT_VAR3(maxLoad * 100.0f));
        // hw.PrintLine("Avg: " FLT_FMT3, FLT_VAR3(avgLoad * 100.0f));
        // hw.PrintLine("Min: " FLT_FMT3, FLT_VAR3(minLoad * 100.0f));
        // // don't spam the serial connection too much
        // System::Delay(500);
	}
}

int main(void)
{
    Init();
    Run();
}
