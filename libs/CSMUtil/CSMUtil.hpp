#ifndef CSMUTIL_HPP
#define CSMUTIL_HPP

namespace csmutil{

static int imap(float value, float from_min, float from_max, int to_min, int to_max){
    return (int)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

template<typename R, typename T, typename F>
static R map(F value, F from_min, F from_max, T to_min, T to_max) {
    return (R)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

};

#endif // CSMUTIL_HPP