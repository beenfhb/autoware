#ifndef _NMS_H_
#define _NMS_H_

#include "MODEL_info.h"
#include "switch_float.h"

// Non_maximum suppression function (extended to detect.cc)
extern FLOAT *dpm_ttic_gpu_nms(FLOAT *boxes, FLOAT overlap, int *num,
                               GPUModel *MO);

#endif /* _NMS_H_ */
