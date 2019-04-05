#include "ompl/base/samplers/deterministic/HaltonSequence.h"

#include <iostream>
#include <cmath>
#include <map>

namespace ompl
{
    namespace base
    {
        HaltonSequence1D::HaltonSequence1D() :
          i_(1), base_(2)
        {
        }

        HaltonSequence1D::HaltonSequence1D(unsigned int base) :
          i_(1), base_(base)
        {

        }


        void HaltonSequence1D::setBase(unsigned int base) {
            base_ = base;
        }

        double HaltonSequence1D::sample() {
            double f=1, r=0;
            unsigned int i = i_;

            while (i>0) {
                f /= base_;
                r += f * (i%base_);
                i = std::floor(i/base_);
            }

            ++i_;
            return r;
        }

        HaltonSequence::HaltonSequence(unsigned int dimensions) :
            DeterministicSequence(dimensions),
            halton_sequences_1d_(dimensions) {
            setBasesToPrimes();
        }

        HaltonSequence::HaltonSequence(unsigned int dimensions, std::vector<unsigned int> bases) :
            DeterministicSequence(dimensions),
            halton_sequences_1d_(dimensions) {
            if(bases.size() != dimensions) {
                std::cout << "Warning: Number of bases does not match dimensions. Using first n primes instead.";
            }
            else {
                for(size_t i=0; i<bases.size(); ++i) {
                    halton_sequences_1d_[i].setBase(bases[i]);
                }
            }
        }

        std::vector<double> HaltonSequence::sample() {
            std::vector<double> samples;
            for(auto & seq : halton_sequences_1d_) {
                samples.push_back(seq.sample());
            }
            return samples;
        }


        void HaltonSequence::setBasesToPrimes() {
            // set the base of the halton sequences to the first n prime numbers, where n is dimensions
            unsigned int primes_found = 0;
            unsigned int current = 2;
            // naive method to finding the prime numbers, but since dimensions
            // is not that large this should normally be fine
            while(primes_found < dimensions_) {
                bool prime = true;
                for(int i=current/2; i >= 2; --i) {
                    if(current % i == 0) {
                        prime = false;
                        break;
                    }
                }
                if(prime == true) {
                    halton_sequences_1d_[primes_found].setBase(current);
                    ++primes_found;
                }
                ++current;
            }
        }
    }
}
