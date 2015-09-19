#ifndef H_SPICONTROL
#define H_SPICONTROL

#ifdef __cplusplus
extern "C" {
#endif

void RF24_init(void);
void RF24_exchangeInfo(int *in, int *out);

#ifdef __cplusplus
}
#endif
#endif
