#include "Grain.h"

using namespace graindelay;

const float Grain::GRAIN_MIN_DURATION = 0.1f;
const float Grain::GRAIN_MAX_DURATION = 10.0f;
const float Grain::PAN_MAX_WIDTH = 1.0f;

float Grain::Process(const float in)
{
    buffer_[write_index_] = in;

    if (!env_.IsRunning())
    {
        audible_ = (rand() * kRandFrac) >= grain_density_ ? 0.0f : 1.0f;
        // grain_start_callback_(audible_ > 0.0f);
        UpdateDuration();
        UpdatePan();
        Trigger();
    }

    const float out = read(read_position_);
    const float level = env_.Process() * amp_ * audible_;

    buffer_[write_index_] += out * level * feedback_;
    write_index_ = (++write_index_) % buffer_size_;
    updateReadPosition();

    return out * level;
}

void Grain::updateReadPosition()
{
    const float updated_position = read_position_ + speed_;
    const size_t x = static_cast<size_t>(updated_position);
    const float f = updated_position - static_cast<float>(x);
    read_position_ = (x % buffer_size_) + f;
}

float Grain::read(size_t position)
{
    return buffer_[position % buffer_size_];
}

// Linear interpolation
float Grain::read(float position)
{
    const int32_t t = static_cast<int32_t>(position);
    const float f = position - static_cast<float>(t);

    const float a = buffer_[t % buffer_size_];
    const float b = buffer_[(t+1) % buffer_size_];

    return a + (b-a) * f;
}

// Hermite interpolation (from here https://github.com/pichenettes/stmlib/blob/master/dsp/dsp.h)
float Grain::readHermite(float position)
{
    const int32_t t = static_cast<int32_t>(position);
    const float f = position - static_cast<float>(t);

    const float xm1 = buffer_[(t - 1) % buffer_size_]; // this line causes issue as a negative index doesn't wrap to the end of the buffer
    const float x0 = buffer_[(t) % buffer_size_];
    const float x1 = buffer_[(t + 1) % buffer_size_];
    const float x2 = buffer_[(t + 2) % buffer_size_];

    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;

    return (((a * f) - b_neg) * f + c) * f + x0;
}

void Grain::UpdateEnvFromDuration()
{
    env_.SetTime(ADENV_SEG_ATTACK, duration_ * 0.5f);
    env_.SetTime(ADENV_SEG_DECAY, duration_ * 0.5f);
}
