// Low-pass filters. Original implementation: Yan Yuanbin / GrassFan Wang.
#ifndef ALG_LPF_HPP
#define ALG_LPF_HPP

struct LowPassFilter1p {
    bool initialized = false;
    float input = 0.0f;
    float output = 0.0f;
    float alpha = 0.0f;
};

struct LowPassFilter2p {
    bool initialized = false;
    float input = 0.0f;
    float output[3] {};
    float alpha[3] {};
};

void lowPassFilter1pInit(LowPassFilter1p* lpf, float alpha);
float lowPassFilter1pUpdate(LowPassFilter1p* lpf, float input);
void lowPassFilter2pInit(LowPassFilter2p* lpf, const float alpha[3]);
float lowPassFilter2pUpdate(LowPassFilter2p* lpf, float input);

#endif // ALG_LPF_HPP
