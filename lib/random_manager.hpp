#ifndef RAND_UTILITIES
#define RAND_UTILITIES

#include <random>
#include <chrono>

namespace randutil {
    //RandomNumberFactory provides an easier interface for generating random numbers without dealing with C-style randomness
    template<typename GeneratorType = std::default_random_engine, typename DistributionType = std::uniform_real_distribution<float>>
    class RandomNumberFactory {
        GeneratorType generator;
        DistributionType distribution;

        public:
        RandomNumberFactory() {
            generator = GeneratorType(std::chrono::high_resolution_clock::now().time_since_epoch().count());
            distribution = DistributionType(0.0f, 1.0f);
        }

        //Produces a random number within the provided range - [lowerBound, upperBound)
        template<typename T>
        T produceRandom(T const& lowerBound, T const& upperBound) {
            return (upperBound - lowerBound) * distribution(generator);
        }
    };
}


#endif