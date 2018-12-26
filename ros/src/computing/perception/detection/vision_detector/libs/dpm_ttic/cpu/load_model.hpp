#ifndef _LOAD_MODEL_H_
#define _LOAD_MODEL_H_

#include "MODEL_info.h"
#include "switch_float.h"

extern MODEL *dpm_ttic_cpu_load_model(FLOAT ratio, const char *com_csv,
                                      const char *root_csv,
                                      const char *part_csv);
extern void dpm_ttic_cpu_free_model(MODEL *MO);

#endif /* _LOAD_MODEL_H_ */
