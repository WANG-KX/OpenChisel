#ifndef PROBABILISTICWEIGHTER_H_
#define PROBABILISTICWEIGHTER_H_

#include "Weighter.h"
#include "math.h"

#define PI 3.1415

namespace chisel
{
    class ProbabilisticWeighter : public Weighter
    {
        public:
            ProbabilisticWeighter() = default;
            virtual ~ProbabilisticWeighter()
            {   
            }   
            virtual float GetWeight(float surfaceDist, float sigma) const
            {
                float exponent = surfaceDist / sigma;
                exponent *= exponent;
                return 1.0f / sqrt(2 * PI * sigma) * exp( - exponent );
            }
    };
    typedef std::shared_ptr<ProbabilisticWeighter> ProbabilisticWeighterPtr;
    typedef std::shared_ptr<const ProbabilisticWeighter> ProbabilisticWeighterConstPtr;

} // namespace chisel 

#endif // PROBABILISTICWEIGHT_H_ 
