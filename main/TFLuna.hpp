// /main/TFLuna.hpp
#ifndef TFLUNA_HPP
#define TFLUNA_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "I2CBus.hpp" // Update this path to the actual relative or absolute path where I2CBus.hpp is located

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class TFLuna {
public:
    TFLuna(I2CBus& bus, uint8_t addr);
    bool readDistance(int16_t& cm);

private:
    I2CBus& bus_;
    uint8_t addr_;
};

#endif // __cplusplus

#endif // TFLUNA_HPP