#ifndef MESHCORE_SIMPLE_REPEATER_QOS_H
#define MESHCORE_SIMPLE_REPEATER_QOS_H

#include <cstdint>
#include <array>
#include <mutex>
#include <cmath>
#include <limits>
#include <vector>

#include "Packet.h"


float distributeBudget(
    size_t numBuckets, 
    float totalAmount, 
    const float* capacity, 
    float* weights, 
    float* distributedAmount
);

class SingleBudgetBucket {
private:
    float budget;
    const float maxBudget;
public:
    SingleBudgetBucket(float initialBudget, float maxBudget) : budget(initialBudget), maxBudget(maxBudget) {}
    float replenish(float amount) {
        if (amount <= 0.0f) return amount;
        float added = std::min(amount, maxBudget - budget);
        budget += added;
        if (added > 0.0f) {
            MESH_DEBUG_PRINTLN("SingleBudgetBucket::replenish(): replenished by %.6f, new budget %.6f", added, budget);
        }
        return amount - added;
    }
    float capacity() {
        return maxBudget - budget;
    }
    float available() {
        return budget;
    }
    void consume(float amount) {
        budget = std::max(0.0f, budget - amount);
    }

    bool tryConsume(float amount, uint8_t hops) {
        // Long distance messages leave some budget for short distance messages.
        // This is effectively a dynamic hop limit that adjusts itself based on the current load.
        // 0 Hops - 0%
        // 1 Hop - 8%
        // 2 Hops - 15%
        // 3 Hops - 22%
        // 4 Hops - 28%
        // 5 Hops - 34%
        // 10 Hops - 56%
        // 15 Hops - 71%
        // 18 Hops - 80%
        // 27 Hops - 90%
        float protectedBudget = std::min(maxBudget - 1.0f, maxBudget * (1.0f - powf(0.92f, hops)));

        float previousBudget = budget;
        if (budget < protectedBudget + amount) {
            return false;
        }
        consume(amount);
        MESH_DEBUG_PRINTLN("SingleBudgetBucket::tryConsume(): consumed %.6f budget, remaining %.6f", previousBudget - budget, budget);
        return true;
    }
};

class HashBuckets {
private:
    static const size_t NUM_BUCKETS = 256;
    float COMMON_MULTIPLIER = 5.0f; // number of concurrently active channels
    const float maxBudget;
    float buckets[NUM_BUCKETS];

    // In addition to per-hash buckets, we also maintain a common bucket that all hashes draw from, to ensure that many small channels don't overload the mesh and prevent short distance message from getting through.
    float commonBucket;

    float forReplenish_weights[NUM_BUCKETS];
    float forReplenish_capacities[NUM_BUCKETS];
    float forReplenish_distributed[NUM_BUCKETS];

public:
    HashBuckets(float initalBudget, float maxBudget) : maxBudget(maxBudget) {
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            buckets[i] = initalBudget;
        }
        commonBucket = initalBudget * COMMON_MULTIPLIER;
    }

    float replenish(float amount) {
        if (amount <= 0.0f) return amount;

        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            forReplenish_weights[i] = 1.0f;
            forReplenish_capacities[i] = maxBudget - buckets[i];
        }
        float remaining = distributeBudget(
            NUM_BUCKETS, 
            amount,
            forReplenish_capacities,
            forReplenish_weights,
            forReplenish_distributed
        );
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            buckets[i] = std::min(maxBudget, buckets[i] + forReplenish_distributed[i]);
            if (forReplenish_distributed[i] > 0.0f) {
                MESH_DEBUG_PRINTLN("HashBuckets::replenish(): bucket %02x replenished by %.6f, new budget %.6f", i, forReplenish_distributed[i], buckets[i]);
            }
        }
        float consumedBudget = amount - remaining;
        // The common bucket is used to give priority to high load local channels over low load channels from other regions.
        // But if there is a local channel with constantly high loads (because of bots) it shouldn't beneift from this mechanism.
        // Making the common bucket recover faster brings it back to 100% while there is still capacity in these high load channels.
        commonBucket = std::min(maxBudget * COMMON_MULTIPLIER, commonBucket + consumedBudget * 1.1f);
        return remaining;
    }

    float capacity() const {
        float totalCapacity = 0.0f;
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            totalCapacity += maxBudget - buckets[i];
        }
        return totalCapacity;
    }

    float available() {
        return commonBucket;
    }

    float available(uint8_t hash) const {
        return std::min(buckets[hash], commonBucket);
    }

    void consume(uint8_t hash, float amount) {
        if (amount <= 0.0f) return;
        buckets[(size_t) hash] = std::max(0.0f, buckets[hash] - amount);
        commonBucket = std::max(0.0f, commonBucket - amount);
    }

    float protectedBudget(float max, uint8_t hops) const {
        // Long distance messages leave some budget for short distance messages.
        // This is effectively a dynamic hop limit that adjusts itself based on the current load.
        return std::min(max - 1.0f, max * (1.0f - powf(0.92f, hops)));
    }

    bool tryConsume(uint8_t hash, float amount, uint8_t hops) {
        if (buckets[hash] < protectedBudget(maxBudget, hops) + amount) {
            return false;
        }
        if (commonBucket < protectedBudget(maxBudget * COMMON_MULTIPLIER, hops) + amount) {
            return false;
        }
        consume(hash, amount);
        MESH_DEBUG_PRINTLN("HashBuckets::tryConsume(): bucket %02x consumed %.6f budget, remaining %.6f", hash, amount, available(hash));
        return true;
    }
};


class PublicKeyBuckets {
private:
    static const size_t NUM_BUCKETS = 1000;
    const float maxBudget;
    float buckets[NUM_BUCKETS];

    float forReplenish_weights[NUM_BUCKETS];
    float forReplenish_capacities[NUM_BUCKETS];
    float forReplenish_distributed[NUM_BUCKETS];

public:
    PublicKeyBuckets(float initalBudget, float maxBudget) : maxBudget(maxBudget) {
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            buckets[i] = initalBudget;
        }
    }

    float replenish(float amount) {
        if (amount <= 0.0f) return amount;

        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            forReplenish_weights[i] = 1.0f;
            forReplenish_capacities[i] = maxBudget - buckets[i];
        }
        
        float remaining = distributeBudget(
            NUM_BUCKETS, 
            amount,
            forReplenish_capacities,
            forReplenish_weights,
            forReplenish_distributed
        );
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            buckets[i] = std::min(maxBudget, buckets[i] + forReplenish_distributed[i]);
        }
        return remaining;
    }

    float capacity() const {
        float totalCapacity = 0.0f;
        for (size_t i = 0; i < NUM_BUCKETS; ++i) {
            totalCapacity += maxBudget - buckets[i];
        }
        return totalCapacity;
    }

    float available(uint32_t hash) const {
        return buckets[hash % NUM_BUCKETS];
    }

    void consume(uint32_t hash, float amount) {
        if (amount <= 0.0f) return;
        buckets[(size_t) hash % NUM_BUCKETS] = std::max(0.0f, buckets[hash % NUM_BUCKETS] - amount);
    }

    bool tryConsume(uint32_t hash, float amount, uint8_t hops) {
        float previousBudget = available(hash);
        if (previousBudget < amount) {
            return false;
        }
        consume(hash, amount);
        MESH_DEBUG_PRINTLN("PublicKeyBuckets::tryConsume(): bucket %02x consumed %.6f budget, remaining %.6f", hash % NUM_BUCKETS, amount, available(hash));
        return true;
    }
};


class Qos {
private:
    mesh::RTCClock* rtcClock;
    uint32_t lastReplenish{0};
    SingleBudgetBucket flood_ack{20.f, 20.f};
    SingleBudgetBucket flood_path{20.f, 20.f};
    HashBuckets flood_groupMessage{60.f, 60.f};
    HashBuckets transportflood_groupMessage{60.f, 60.f};
    HashBuckets flood_textMessage{30.f, 30.f};
    PublicKeyBuckets flood_companionAdvert{3.f, 3.f};
    PublicKeyBuckets flood_repeaterAdvert{1.f, 1.f};
    SingleBudgetBucket flood_anonRequest{10.f, 30.f};
    SingleBudgetBucket flood_request{10.f, 30.f};
    SingleBudgetBucket flood_response{10.f, 50.f};
    SingleBudgetBucket flood_other{10.f, 10.f};
    SingleBudgetBucket direct_trace{10.f, 10.f};
    SingleBudgetBucket direct_other{40.f, 40.f};
    uint8_t publicKeyRandomOffset;
    double packetsPerHour = 90.0;


    float forReplenish_distributed[12];
    float forReplenish_weights[12];
    float forReplenish_capacities[12];
public:
    Qos(mesh::RTCClock* rtcClock) : rtcClock(rtcClock) {
        // By picking random bytes of the public key the collisions will be different on each repeater and the advert has the chance to take a different route.
        publicKeyRandomOffset = random() % (32 - 4); // public key is 32 bytes and we take 4 bytes
    }

    void setPacketsPerHours(double value) {
        packetsPerHour = value;
    }

    void replenish() {
        uint32_t currentTimeSeconds = rtcClock->getCurrentTime();
        uint32_t elapsedTime = currentTimeSeconds - lastReplenish;
        if (elapsedTime <= 0 || elapsedTime > 60L * 60L) {
            lastReplenish = currentTimeSeconds;
            return;    
        }
        double secondsBetweenPackets = 60.0 * 60.0 / packetsPerHour;
        float budget = ((double)elapsedTime) / secondsBetweenPackets;
        MESH_DEBUG_PRINTLN("Qos::replenish(): rate %.1f, elapsed time %d seconds, replenishing budget by %.6f", packetsPerHour, elapsedTime, budget);
        if (budget <= 0.5f) return; // float isn't precise enough for tiny increments
        lastReplenish = currentTimeSeconds;
        replenish(budget);
    }

    void replenish(float amount) {
        forReplenish_capacities[0] = flood_ack.capacity();
        forReplenish_capacities[1] = flood_path.capacity();
        forReplenish_capacities[2] = flood_groupMessage.capacity();
        forReplenish_capacities[3] = transportflood_groupMessage.capacity();
        forReplenish_capacities[4] = flood_textMessage.capacity();
        forReplenish_capacities[5] = flood_companionAdvert.capacity();
        forReplenish_capacities[6] = flood_repeaterAdvert.capacity();
        forReplenish_capacities[7] = flood_anonRequest.capacity();
        forReplenish_capacities[8] = flood_request.capacity();
        forReplenish_capacities[9] = flood_response.capacity();
        forReplenish_capacities[10] = flood_other.capacity();
        forReplenish_capacities[11] = direct_trace.capacity();
        forReplenish_capacities[12] = direct_other.capacity();

        forReplenish_weights[0] = 5.0f; // flood_ack
        forReplenish_weights[1] = 2.0f; // flood_path
        forReplenish_weights[2] = 25.0f; // flood_groupMessage
        forReplenish_weights[3] = 25.0f; // transportflood_groupMessage
        forReplenish_weights[4] = 20.0f; // flood_textMessage
        forReplenish_weights[5] = 1.0f; // flood_companionAdvert
        forReplenish_weights[6] = 1.0f; // flood_repeaterAdvert
        forReplenish_weights[7] = 0.25f; // flood_anonRequest
        forReplenish_weights[8] = 0.75f; // flood_request
        forReplenish_weights[9] = 1.0f; // flood_response
        forReplenish_weights[10] = 5.0f; // flood_other
        forReplenish_weights[11] = 0.5f; // direct_trace
        forReplenish_weights[12] = 10.0f; // direct_other

        distributeBudget(
            13, 
            amount,
            forReplenish_capacities,
            forReplenish_weights,
            forReplenish_distributed
        );
        
        flood_ack.replenish(forReplenish_distributed[0]);
        flood_path.replenish(forReplenish_distributed[1]);
        flood_groupMessage.replenish(forReplenish_distributed[2]);
        transportflood_groupMessage.replenish(forReplenish_distributed[3]);
        flood_textMessage.replenish(forReplenish_distributed[4]);
        flood_companionAdvert.replenish(forReplenish_distributed[5]);
        flood_repeaterAdvert.replenish(forReplenish_distributed[6]);
        flood_anonRequest.replenish(forReplenish_distributed[7]);
        flood_request.replenish(forReplenish_distributed[8]);
        flood_response.replenish(forReplenish_distributed[9]);
        flood_other.replenish(forReplenish_distributed[10]);
        direct_trace.replenish(forReplenish_distributed[11]);
        direct_other.replenish(forReplenish_distributed[12]);

        MESH_DEBUG_PRINTLN("Qos::replenish(): current budgets:");
        MESH_DEBUG_PRINTLN(" flood_ack              = %.6f", flood_ack.available());
        MESH_DEBUG_PRINTLN(" flood_path             = %.6f", flood_path.available());
        MESH_DEBUG_PRINTLN(" flood_groupMessage     = %.6f", flood_groupMessage.available());
        MESH_DEBUG_PRINTLN(" tflood_groupMessage    = %.6f", transportflood_groupMessage.available());
        MESH_DEBUG_PRINTLN(" flood_textMessage      = %.6f", flood_textMessage.available());
        MESH_DEBUG_PRINTLN(" flood_anonRequest      = %.6f", flood_anonRequest.available());
        MESH_DEBUG_PRINTLN(" flood_request          = %.6f", flood_request.available());
        MESH_DEBUG_PRINTLN(" flood_response         = %.6f", flood_response.available());
        MESH_DEBUG_PRINTLN(" flood_other            = %.6f", flood_other.available());
        MESH_DEBUG_PRINTLN(" flood_companionAdvert ~= %.6f", flood_companionAdvert.capacity());
        MESH_DEBUG_PRINTLN(" flood_repeaterAdvert  ~= %.6f", flood_repeaterAdvert.capacity());
        MESH_DEBUG_PRINTLN(" direct_trace           = %.6f", direct_trace.available());
        MESH_DEBUG_PRINTLN(" direct_other           = %.6f", direct_other.available());

    }

    bool tryConsume(const mesh::Packet *packet) {
        replenish();
        if (packet->isRouteFlood()) {
            switch (packet->getPayloadType()) {
                case PAYLOAD_TYPE_GRP_TXT:
                    if (packet->hasTransportCodes()) {
                        return flood_groupMessage.tryConsume(packet->payload[0], 1.0f, packet->path_len);
                    } else {
                        return transportflood_groupMessage.tryConsume(packet->payload[0], 1.0f, packet->path_len);
                    }
                case PAYLOAD_TYPE_RESPONSE:
                    return flood_response.tryConsume(1.0f, packet->path_len);
                case PAYLOAD_TYPE_REQ:
                    return flood_request.tryConsume(1.0f, packet->path_len);
                case PAYLOAD_TYPE_ADVERT:
                    if (packet->payload_len > 100) {
                        uint32_t publicKeyHash = packet->payload[publicKeyRandomOffset] | (packet->payload[publicKeyRandomOffset + 1] << 8) | (packet->payload[publicKeyRandomOffset + 2] << 16) | (packet->payload[publicKeyRandomOffset + 3] << 24);
                        uint8_t flags = packet->payload[100];
                        bool isCompanion = (flags & 0xf) == 1;
                        if (isCompanion) {
                            return flood_companionAdvert.tryConsume(publicKeyHash, 1.0f, packet->path_len);
                        } else {
                            return flood_repeaterAdvert.tryConsume(publicKeyHash, 1.0f, packet->path_len);
                        }
                    }
                case PAYLOAD_TYPE_TXT_MSG:
                    return flood_textMessage.tryConsume(packet->payload[1], 1.0f, packet->path_len);    
                case PAYLOAD_TYPE_PATH:
                    return flood_path.tryConsume(1.0f, packet->path_len);   
                case PAYLOAD_TYPE_ANON_REQ:
                    return flood_anonRequest.tryConsume(1.0f, packet->path_len);
                case PAYLOAD_TYPE_ACK:
                    return flood_ack.tryConsume(1.0f, packet->path_len);
                default:
                    return flood_other.tryConsume(1.0f, packet->path_len);
            }
        } else {
            switch (packet->getPayloadType()) {
                case PAYLOAD_TYPE_TRACE:
                    return direct_trace.tryConsume(1.0f, packet->path_len);
                default:
                    return direct_other.tryConsume(1.0f, packet->path_len);
            }
        }
    }
};

#endif // MESHCORE_SIMPLE_REPEATER_QOS_H
