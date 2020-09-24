#ifndef SDM_H
#define SDM_H

#ifdef __cplusplus
extern "C"
{
#endif

void Am_Sigma_Delta_Init(uint8_t Order, uint8_t Osr);
void Am_Sigma_Delta(uint16_t *out, int16_t in, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // SDM_H
