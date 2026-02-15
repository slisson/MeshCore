#include "Qos.h"

float distributeBudget(
    size_t numBuckets, 
    float totalAmount, 
    const float* capacity, 
    float* weights, 
    float* distributedAmount
) {
    float sumOfWeights = 0.0f;
    for (size_t i = 0; i < numBuckets; ++i) {
        if (capacity[i] <= 0.0f) {
            weights[i] = 0.0f;
        }
        sumOfWeights += weights[i];
        distributedAmount[i] = 0.0f;
    }
    
    float remaining = totalAmount;
    for (uint8_t timeout = 10; timeout > 0; timeout--) {
        float currentIterationAmount = remaining;
        bool anyUpdated = false;
        for (size_t i = 0; i < numBuckets; ++i) {
            float previousDistributedAmount = distributedAmount[i];
            float relativeWeight = std::min(1.0f, weights[i] / sumOfWeights);
            if (relativeWeight <= 0.0f) continue;
            float amountForBucket = currentIterationAmount * relativeWeight;
            if (distributedAmount[i] + amountForBucket >= capacity[i]) {
                remaining -= capacity[i] - distributedAmount[i];
                distributedAmount[i] = capacity[i];
                sumOfWeights -= weights[i];
                weights[i] = 0.0f;
            } else {
                remaining -= amountForBucket;
                distributedAmount[i] += amountForBucket;
            }
            if (distributedAmount[i] != previousDistributedAmount) {
                anyUpdated = true;
            }
        }
        if (!anyUpdated) break;
        if (sumOfWeights <= 0.0f) break;
    }
    return remaining;
}