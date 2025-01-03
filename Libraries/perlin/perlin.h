#ifndef PERLIN_H
#define PERLIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

float perlin(uint32_t fx, uint32_t fy,
             uint32_t canvas_size_x, uint32_t canvas_size_y,
             uint32_t cell_size);

#ifdef __cplusplus
}
#endif

#endif /* PERLIN_H */