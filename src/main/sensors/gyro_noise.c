/*
 * gyro_noise.c - Wide Spectrum Noise Generation for Gyroscope Testing
 * 
 * Implementation of noise generation functions for simulating real-world
 * gyroscope sensor conditions for testing and validation of filtering algorithms.
 */

#include "gyro_noise.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Example usage function for reference
#if false
void exampleGyroFilterWithNoise(float gyroSamples[3])
{
    // Initialize noise generator (typically done once at startup)
    static bool noiseInitialized = false;
    if (!noiseInitialized) {
        gyroNoiseInit(12345); // Use fixed seed for reproducible results
        
        // Configure noise parameters
        gyroNoiseConfig_t config = GYRO_NOISE_CONFIG_DEFAULT();
        config.amplitude = 1.5f;
        config.enabledTypes = GYRO_NOISE_WHITE | GYRO_NOISE_PINK | GYRO_NOISE_BROWN;
        gyroNoiseSetConfig(&config);
        
        noiseInitialized = true;
    }
    
    // Add noise to each axis
    for (int axis = 0; axis < 3; axis++) {
        gyroSamples[axis] = GYRO_NOISE_INJECT(axis, gyroSamples[axis]);
    }
}
#endif

// Static variables for noise generation
static uint32_t noiseState = 1;
static float noiseBuffer[GYRO_NOISE_MAX_AXES][GYRO_NOISE_BUFFER_SIZE];
static gyroNoiseConfig_t noiseConfig;
static bool isInitialized = false;

// Pink noise filter coefficients (Paul Kellet's algorithm)
static const float pinkCoeff[6] = {
    0.99886f, 0.99332f, 0.96900f, 0.86650f, 0.55000f, -0.7616f
};

static const float pinkGain[7] = {
    0.0555179f, 0.0750759f, 0.1538520f, 0.3104856f, 
    0.5329522f, 0.0168980f, 0.115926f
};

/**
 * @brief Initialize the gyro noise generator with a seed value
 */
void gyroNoiseInit(uint32_t seed)
{
    noiseState = seed ? seed : 1;
    
    // Clear noise buffers
    memset(noiseBuffer, 0, sizeof(noiseBuffer));
    
    // Initialize default configuration
    noiseConfig.amplitude = GYRO_NOISE_DEFAULT_AMPLITUDE;
    noiseConfig.whiteScale = GYRO_NOISE_WHITE_SCALE;
    noiseConfig.pinkScale = GYRO_NOISE_PINK_SCALE;
    noiseConfig.brownScale = GYRO_NOISE_BROWN_SCALE;
    noiseConfig.burstScale = GYRO_NOISE_BURST_SCALE;
    noiseConfig.burstProbability = GYRO_NOISE_BURST_PROBABILITY;
    noiseConfig.enabledTypes = GYRO_NOISE_ALL;
    noiseConfig.temperatureDependent = false;
    noiseConfig.quantizationNoise = false;
    
    isInitialized = true;
}

/**
 * @brief Set noise configuration parameters
 */
void gyroNoiseSetConfig(const gyroNoiseConfig_t *config)
{
    if (config) {
        memcpy(&noiseConfig, config, sizeof(gyroNoiseConfig_t));
    }
}

/**
 * @brief Get current noise configuration
 */
void gyroNoiseGetConfig(gyroNoiseConfig_t *config)
{
    if (config) {
        memcpy(config, &noiseConfig, sizeof(gyroNoiseConfig_t));
    }
}

/**
 * @brief Generate white noise with specified amplitude
 */
float gyroNoiseGenerateWhite(float amplitude)
{
    if (!isInitialized) {
        gyroNoiseInit(0);
    }
    
    uint32_t randVal = gyroNoiseFastRandom(&noiseState);
    return amplitude * gyroNoiseNormalizeRandom(randVal);
}

/**
 * @brief Generate pink noise (1/f) for a specific axis
 */
float gyroNoiseGeneratePink(int axis, float amplitude)
{
    if (!isInitialized || axis >= GYRO_NOISE_MAX_AXES) {
        return 0.0f;
    }
    
    float white = gyroNoiseGenerateWhite(1.0f);
    
    // Pink noise filter bank using Paul Kellet's algorithm
    noiseBuffer[axis][0] = pinkCoeff[0] * noiseBuffer[axis][0] + white * pinkGain[0];
    noiseBuffer[axis][1] = pinkCoeff[1] * noiseBuffer[axis][1] + white * pinkGain[1];
    noiseBuffer[axis][2] = pinkCoeff[2] * noiseBuffer[axis][2] + white * pinkGain[2];
    noiseBuffer[axis][3] = pinkCoeff[3] * noiseBuffer[axis][3] + white * pinkGain[3];
    noiseBuffer[axis][4] = pinkCoeff[4] * noiseBuffer[axis][4] + white * pinkGain[4];
    noiseBuffer[axis][5] = pinkCoeff[5] * noiseBuffer[axis][5] - white * pinkGain[5];
    
    float pink = noiseBuffer[axis][0] + noiseBuffer[axis][1] + noiseBuffer[axis][2] + 
                 noiseBuffer[axis][3] + noiseBuffer[axis][4] + noiseBuffer[axis][5] + 
                 noiseBuffer[axis][6] + white * 0.5362f;
    
    noiseBuffer[axis][6] = white * pinkGain[6];
    
    return pink * amplitude * 0.11f; // Scale to reasonable amplitude
}

/**
 * @brief Generate brown noise (1/f²) for a specific axis
 */
float gyroNoiseGenerateBrown(int axis, float amplitude)
{
    if (!isInitialized || axis >= GYRO_NOISE_MAX_AXES) {
        return 0.0f;
    }
    
    float white = gyroNoiseGenerateWhite(1.0f);
    noiseBuffer[axis][7] += white * 0.02f;
    
    // Prevent drift
    noiseBuffer[axis][7] *= 0.9999f;
    
    return noiseBuffer[axis][7] * amplitude;
}

/**
 * @brief Generate burst noise (random spikes)
 */
float gyroNoiseGenerateBurst(float amplitude, float burstScale, uint16_t probability)
{
    if (!isInitialized) {
        return 0.0f;
    }
    
    uint32_t randVal = gyroNoiseFastRandom(&noiseState);
    if ((randVal & 0xFFFF) < probability) {
        return gyroNoiseGenerateWhite(amplitude * burstScale);
    }
    
    return 0.0f;
}

/**
 * @brief Generate temperature-dependent noise
 */
float gyroNoiseGenerateTemperature(float baseValue, float tempCoeff)
{
    if (!isInitialized) {
        return 0.0f;
    }
    
    return gyroNoiseGenerateWhite(tempCoeff * fabsf(baseValue));
}

/**
 * @brief Generate quantization noise
 */
float gyroNoiseGenerateQuantization(float amplitude)
{
    if (!isInitialized) {
        return 0.0f;
    }
    
    return gyroNoiseGenerateWhite(amplitude);
}

/**
 * @brief Generate wide spectrum noise for a specific axis
 */
float gyroNoiseGenerateWideSpectrum(int axis, float amplitude)
{
    if (!isInitialized || axis >= GYRO_NOISE_MAX_AXES) {
        return 0.0f;
    }
    
    float totalNoise = 0.0f;
    
    // Add white noise (high frequency content)
    if (noiseConfig.enabledTypes & GYRO_NOISE_WHITE) {
        totalNoise += gyroNoiseGenerateWhite(amplitude * noiseConfig.whiteScale);
    }
    
    // Add pink noise (mid frequency content)
    if (noiseConfig.enabledTypes & GYRO_NOISE_PINK) {
        totalNoise += gyroNoiseGeneratePink(axis, amplitude * noiseConfig.pinkScale);
    }
    
    // Add brown noise (low frequency content)
    if (noiseConfig.enabledTypes & GYRO_NOISE_BROWN) {
        totalNoise += gyroNoiseGenerateBrown(axis, amplitude * noiseConfig.brownScale);
    }
    
    // Add burst noise (random spikes)
    if (noiseConfig.enabledTypes & GYRO_NOISE_BURST) {
        totalNoise += gyroNoiseGenerateBurst(amplitude, noiseConfig.burstScale, 
                                           noiseConfig.burstProbability);
    }
    
    return totalNoise;
}

/**
 * @brief Add noise to gyro sample with current configuration
 */
float gyroNoiseAddToSample(int axis, float gyroValue)
{
    if (!isInitialized || axis >= GYRO_NOISE_MAX_AXES) {
        return gyroValue;
    }
    
    float noisyValue = gyroValue;
    
    // Add wide spectrum noise
    noisyValue += gyroNoiseGenerateWideSpectrum(axis, noiseConfig.amplitude);
    
    // Add temperature-dependent noise if enabled
    if (noiseConfig.temperatureDependent) {
        noisyValue += gyroNoiseGenerateTemperature(gyroValue, 0.01f);
    }
    
    // Add quantization noise if enabled
    if (noiseConfig.quantizationNoise) {
        noisyValue += gyroNoiseGenerateQuantization(0.5f);
    }
    
    return noisyValue;
}

/**
 * @brief Reset noise filters and buffers
 */
void gyroNoiseReset(void)
{
    memset(noiseBuffer, 0, sizeof(noiseBuffer));
}

/**
 * @brief Get current PRNG state (for debugging/testing)
 */
uint32_t gyroNoiseGetPRNGState(void)
{
    return noiseState;
}

/**
 * @brief Set PRNG state (for debugging/testing)
 */
void gyroNoiseSetPRNGState(uint32_t state)
{
    noiseState = state ? state : 1;
}
