#ifndef K210_FACE_H_
#define K210_FACE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint16_t center_x;
    uint16_t center_y;
    uint8_t class_id;
    uint16_t score_permille;
    bool valid;
    uint32_t last_update_ms;
} K210FaceDetection;

void K210Face_Init(void);
void K210Face_Task(void);
void K210Face_Tick1ms(void);
bool K210Face_SendDMA(const uint8_t *data, uint16_t len);
bool K210Face_SendStringDMA(const char *text);
bool K210Face_IsTxBusy(void);
void K210Face_SetTrackingEnabled(bool enabled);
bool K210Face_GetLatest(K210FaceDetection *detection);
uint32_t K210Face_GetRxByteCount(void);
uint32_t K210Face_GetRxLineCount(void);
uint32_t K210Face_GetRxEdgeCount(void);
uint32_t K210Face_GetRxErrorCount(void);
uint32_t K210Face_GetRxDmaBlockCount(void);
uint32_t K210Face_GetTxDmaDoneCount(void);
uint32_t K210Face_GetValidCount(void);
uint32_t K210Face_GetParseErrorCount(void);
uint32_t K210Face_GetTrackCommandCount(void);
int32_t K210Face_GetLastErrorX(void);
int32_t K210Face_GetLastErrorY(void);
uint8_t K210Face_GetLastByte(void);
uint8_t K210Face_GetRxPinLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* K210_FACE_H_ */
