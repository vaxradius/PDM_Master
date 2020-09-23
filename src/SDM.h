#ifndef SDM_H
#define SDM_H

#ifdef __cplusplus
extern "C"
{
#endif

void Sigma_Delta_Modulator(uint16_t *out, int16_t in, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // SDM_H
