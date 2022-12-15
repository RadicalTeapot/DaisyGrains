#pragma once
#ifndef GRAIN_H
#define GRAIN_H
#include <stdint.h>
#include "daisysp.h"
#ifdef __cplusplus

using namespace daisysp;

namespace graindelay
{
    typedef void (*grain_start_callback)(bool);
class Grain
{
    public:
        Grain() {}
        ~Grain() {}

        void Init(const float sample_rate, float* buffer, const size_t size)
        {
            buffer_ = buffer;
            buffer_size_ = size - INTERPOLATION_TAIL;
            for (size_t i = 0; i < size; i++)
                buffer_[i] = 0;
            write_index_ = 0;
            read_position_ = 0;

            feedbackSvf_.Init(sample_rate);
            env_.Init(sample_rate);

            audible_ = 0.0f;
            speed_ = 1.0f;
            amp_ = 0.0f;
            grain_density_ = 0.0f;
            feedback_ = 0.0f;

            grain_start_callback empty_callback {};
            grain_start_callback_ = empty_callback;
        }

        inline void SetSpeed(const float speed)
        {
            speed_ = speed;
        }

        void SetAmp(const float amp)
        {
            amp_ = fclamp(amp, 0.0f, 1.0f);
        }

        inline void SetFeedback(const float feedback)
        {
            feedback_ = fclamp(feedback, 0.0f, 1.0f);
            feedbackSvf_.SetFreq(20.0f + 100.0f * feedback_ * feedback_);   // Formula from https://github.com/pichenettes/eurorack/blob/master/clouds/dsp/granular_processor.cc
        }

        inline void Trigger()
        {
            env_.Trigger();
        }

        inline void SetDuration(const float length)
        {
            nextDuration_ = length;
        }

        inline void SetGrainStartCallback(grain_start_callback grain_start_callback)
        {
            grain_start_callback_ = grain_start_callback;
        }

        inline void SetGrainDensity(const float grain_density)
        {
            grain_density_ = fclamp(grain_density, 0.0f, 1.0f);
        }

        inline float GetPan() {return pan_;}

        float Process(const float in);

    private:
        float read(size_t position);
        float read(float position);
        float readHermite(float position);
        void updateReadPosition();

        float* buffer_;
        size_t buffer_size_;
        uint32_t write_index_;
        float read_position_;
        float grain_density_;
        float speed_, amp_, duration_, pan_, feedback_, nextDuration_;
        float audible_;
        Svf feedbackSvf_;
        static const float GRAIN_MIN_DURATION;
        static const float GRAIN_MAX_DURATION;
        static const float PAN_MAX_WIDTH;
        static const int32_t INTERPOLATION_TAIL;

        AdEnv env_;
        grain_start_callback grain_start_callback_;

        void UpdateEnvFromDuration();

        inline void UpdateDuration()
        {
            const float dialedDuration = fmap(nextDuration_, GRAIN_MIN_DURATION, GRAIN_MAX_DURATION);
            // Add +-5% variation to dialed in value
            const float variation = fmap(rand() * kRandFrac, dialedDuration * -1, dialedDuration) * 0.05f;
            duration_ = fclamp(dialedDuration + variation, GRAIN_MIN_DURATION, GRAIN_MAX_DURATION);
            UpdateEnvFromDuration();
        }

        inline void UpdatePan()
        {
            pan_ = fmap(rand() * kRandFrac, -1 * PAN_MAX_WIDTH, PAN_MAX_WIDTH);
        }
};
}

#endif
#endif
