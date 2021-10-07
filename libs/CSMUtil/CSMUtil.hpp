#ifndef CSMUTIL_HPP
#define CSMUTIL_HPP

class CSMUtil{
    public:
        CSMUtil() = default;
        ~CSMUtil() = default;

        static int imap(float value, float from_min, float from_max, int to_min, int to_max){
            return (int)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
        }
};

#endif // CSMUTIL_HPP