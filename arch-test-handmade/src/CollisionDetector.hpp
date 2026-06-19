#ifndef COLLISION_DETECTOR_HPP
#define COLLISION_DETECTOR_HPP

#include <iostream>

template <class Phase1, class Phase2>
struct CollisionDetector {
    Phase1 p1;
    Phase2 p2;

    CollisionDetector(const Phase1::Config& p1Config, const Phase2::Config& p2Config) 
        : p1(p1Config), p2(p2Config) {}

    void dcd() {
        std::cout << " - CollisionDetector.dcd()" << std::endl;
        p1.invoke();
        p2.invoke();
    }
    void ccd() {
        std::cout << " - CollisionDetector.ccd()" << std::endl;
        p1.invoke();
        p2.invoke();

    }
};

#endif // !COLLISION_DETECTOR_HPP
