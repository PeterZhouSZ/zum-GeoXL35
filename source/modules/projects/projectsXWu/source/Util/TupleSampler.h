#ifndef TupleSampler_H
#define TupleSampler_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "boost/random.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template< typename T_=mpcard, size_t N_=2 >
class TupleContainer
{
public:
    typedef boost::array< T_, N_ > ElemT;

public:
    bool Insert(ElemT& val)
    {
        if (mTuples.find(val) == mTuples.end()) {
            mTuples.insert(val);
            return true;
        }
        return false;
    }

private:
    struct hash_value : std::unary_function< ElemT, boost::uint64_t > {
        boost::uint64_t operator() (ElemT const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };
    boost::unordered_set< ElemT, hash_value > mTuples;
};

template< typename T_=mpcard, size_t N_=2 >
class TupleSampler
{
public:
    typedef boost::array< T_, N_ > ElemT;

public:
    TupleSampler(T_ NM) : numMax_(NM), gen_(0, NM) {}
    bool Sample(ElemT& val)
    {
        mpcard ntry = numMax_ * N_;
        while (0 < ntry--) {
            for (size_t ii = 0; ii < N_; ++ii) {
                val[ii] = gen_(rng_);
            }
            if (container_.Insert(val)) return true;
        }
        return false;
    }

    bool SampleUniq(ElemT& val)
    {
        set<T_> uk;
        mpcard ktry = N_;
        while (0 < ktry--) {
            uk.clear();
            for (size_t ii = 0; ii < N_; ++ii) {
                mpcard ntry = numMax_;
                bool bf = false;
                while (0 < ntry--) {
                    val[ii] = gen_(rng_);
                    if (uk.find(val[ii]) == uk.end()) {
                        bf = true;
                        break;
                    }
                }
                if (!bf) return false;
            }
            if (container_.Insert(val)) return true;
        }
        return false;
    }

    bool SampleUniqSort(ElemT& val)
    {
        mpcard ntry = numMax_;
        while (0 < ntry--) {
            if (!SampleUniq(val)) continue;
            sort(val.begin(), val.end());
            if (container_.Insert(val)) return true;
        }
        return false;
    }

private:
    TupleContainer<T_, N_> container_;
    T_ numMax_;
    boost::random::uniform_int_distribution<T_> gen_;
    boost::random::mt19937 rng_;
};

typedef TupleSampler<> PairSampler;
typedef PairSampler::ElemT PairT;

#endif
