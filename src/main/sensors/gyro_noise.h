/*
 * gyro_noise.h - Wide Spectrum Noise Generation for Gyroscope Testing
 * 
 * This header provides functions for generating various types of noise
 * to simulate real-world gyroscope sensor conditions for testing and
 * validation of filtering algorithms.
 */

#ifndef GYRO_NOISE_H
#define GYRO_NOISE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration defines
#ifndef GYRO_NOISE_MAX_AXES
#define GYRO_NOISE_MAX_AXES 3
#endif

#ifndef GYRO_NOISE_BUFFER_SIZE
#define GYRO_NOISE_BUFFER_SIZE 8
#endif

// Default noise amplitudes (can be overridden)
#ifndef GYRO_NOISE_DEFAULT_AMPLITUDE
#define GYRO_NOISE_DEFAULT_AMPLITUDE 2.0f
#endif

#ifndef GYRO_NOISE_WHITE_SCALE
#define GYRO_NOISE_WHITE_SCALE 0.3f
#endif

#ifndef GYRO_NOISE_PINK_SCALE
#define GYRO_NOISE_PINK_SCALE 0.5f
#endif

#ifndef GYRO_NOISE_BROWN_SCALE
#define GYRO_NOISE_BROWN_SCALE 0.2f
#endif

#ifndef GYRO_NOISE_BURST_SCALE
#define GYRO_NOISE_BURST_SCALE 5.0f
#endif

#ifndef GYRO_NOISE_BURST_PROBABILITY
#define GYRO_NOISE_BURST_PROBABILITY 10  // out of 65536 (0.015%)
#endif

// Noise type flags for selective noise generation
typedef enum {
    GYRO_NOISE_WHITE = (1 << 0),
    GYRO_NOISE_PINK  = (1 << 1), 
    GYRO_NOISE_BROWN = (1 << 2),
    GYRO_NOISE_BURST = (1 << 3),
    GYRO_NOISE_ALL   = 0xFF
} gyroNoiseType_e;

// Noise configuration structure
typedef struct {
    float amplitude;
    float whiteScale;
    float pinkScale;
    float brownScale;
    float burstScale;
    uint16_t burstProbability;
    gyroNoiseType_e enabledTypes;
    bool temperatureDependent;
    bool quantizationNoise;
} gyroNoiseConfig_t;

// Function declarations

/**
 * @brief Initialize the gyro noise generator with a seed value
 * @param seed Random seed value (use 0 for default seed)
 */
void gyroNoiseInit(uint32_t seed);

/**
 * @brief Set noise configuration parameters
 * @param config Pointer to configuration structure
 */
void gyroNoiseSetConfig(const gyroNoiseConfig_t *config);

/**
 * @brief Get current noise configuration
 * @param config Pointer to configuration structure to fill
 */
void gyroNoiseGetConfig(gyroNoiseConfig_t *config);

/**
 * @brief Generate wide spectrum noise for a specific axis
 * @param axis Axis index (0=X, 1=Y, 2=Z)
 * @param amplitude Noise amplitude multiplier
 * @return Generated noise value
 */
float gyroNoiseGenerateWideSpectrum(int axis, float amplitude);

/**
 * @brief Generate white noise with specified amplitude
 * @param amplitude Noise amplitude
 * @return Generated white noise value
 */
float gyroNoiseGenerateWhite(float amplitude);

/**
 * @brief Generate pink noise (1/f) for a specific axis
 * @param axis Axis index (0=X, 1=Y, 2=Z)
 * @param amplitude Noise amplitude
 * @return Generated pink noise value
 */
float gyroNoiseGeneratePink(int axis, float amplitude);

/**
 * @brief Generate brown noise (1/f²) for a specific axis
 * @param axis Axis index (0=X, 1=Y, 2=Z)
 * @param amplitude Noise amplitude
 * @return Generated brown noise value
 */
float gyroNoiseGenerateBrown(int axis, float amplitude);

/**
 * @brief Generate burst noise (random spikes)
 * @param amplitude Base noise amplitude
 * @param burstScale Burst amplitude multiplier
 * @param probability Burst probability (0-65535)
 * @return Generated burst noise value
 */
float gyroNoiseGenerateBurst(float amplitude, float burstScale, uint16_t probability);

/**
 * @brief Generate temperature-dependent noise
 * @param baseValue Base gyro value for temperature scaling
 * @param tempCoeff Temperature coefficient
 * @return Generated temperature noise value
 */
float gyroNoiseGenerateTemperature(float baseValue, float tempCoeff);

/**
 * @brief Generate quantization noise
 * @param amplitude Quantization noise amplitude
 * @return Generated quantization noise value
 */
float gyroNoiseGenerateQuantization(float amplitude);

/**
 * @brief Add noise to gyro sample with current configuration
 * @param axis Axis index (0=X, 1=Y, 2=Z)
 * @param gyroValue Original gyro value
 * @return Gyro value with added noise
 */
float gyroNoiseAddToSample(int axis, float gyroValue);

/**
 * @brief Reset noise filters and buffers
 */
void gyroNoiseReset(void);

/**
 * @brief Get current PRNG state (for debugging/testing)
 * @return Current PRNG state
 */
uint32_t gyroNoiseGetPRNGState(void);

/**
 * @brief Set PRNG state (for debugging/testing)
 * @param state New PRNG state
 */
void gyroNoiseSetPRNGState(uint32_t state);

// Inline utility functions for performance-critical code

/**
 * @brief Fast pseudo-random number generator (inline for performance)
 * @return Pseudo-random 32-bit value
 */
static inline uint32_t gyroNoiseFastRandom(uint32_t *state) {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    return *state;
}

/**
 * @brief Convert random value to normalized float [-1.0, 1.0]
 * @param randVal Random 32-bit value
 * @return Normalized float value
 */
static inline float gyroNoiseNormalizeRandom(uint32_t randVal) {
    return ((float)randVal / (float)UINT32_MAX - 0.5f) * 2.0f;
}

// Macro for easy noise injection in filter functions
#ifdef GYRO_NOISE_INJECTION_ENABLED
#define GYRO_NOISE_INJECT(axis, gyroValue) gyroNoiseAddToSample(axis, gyroValue)
#else
#define GYRO_NOISE_INJECT(axis, gyroValue) (gyroValue)
#endif

// Default configuration initializer
#define GYRO_NOISE_CONFIG_DEFAULT() { \
    .amplitude = GYRO_NOISE_DEFAULT_AMPLITUDE, \
    .whiteScale = GYRO_NOISE_WHITE_SCALE, \
    .pinkScale = GYRO_NOISE_PINK_SCALE, \
    .brownScale = GYRO_NOISE_BROWN_SCALE, \
    .burstScale = GYRO_NOISE_BURST_SCALE, \
    .burstProbability = GYRO_NOISE_BURST_PROBABILITY, \
    .enabledTypes = GYRO_NOISE_ALL, \
    .temperatureDependent = false, \
    .quantizationNoise = false \
}

#ifdef __cplusplus
}
#endif

#endif // GYRO_NOISE_H