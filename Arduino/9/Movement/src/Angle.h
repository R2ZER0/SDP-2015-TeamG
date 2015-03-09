/* Angle.h */

#define PI (3.1415927)

class Angle {
private:
    float radians; // Between -PI and PI
    
public:
    static float normalise(float a);
    
    inline Angle(void) : radians(0.0f) {}
    inline Angle(float rad) { radians = normalise(rad); }
    
    inline float getRadians() const { return radians; }
};


inline Angle operator+(const Angle& lhs, float rhs) {
    return Angle(lhs.getRadians() + rhs);
}

inline Angle operator+(const Angle& lhs, Angle& rhs) {
    return (lhs + rhs.getRadians());
}