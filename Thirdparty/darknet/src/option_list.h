#ifndef OPTION_list_H
#define OPTION_list_H

#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char *key;
    char *val;
    int used;
} kvp;


listc *read_data_cfg(char *filename);

int read_option(char *s, listc *options);

void option_insert(listc *l, char *key, char *val);

char *option_find(listc *l, char *key);

char *option_find_str(listc *l, char *key, char *def);

int option_find_int(listc *l, char *key, int def);

int option_find_int_quiet(listc *l, char *key, int def);

float option_find_float(listc *l, char *key, float def);

float option_find_float_quiet(listc *l, char *key, float def);

void option_unused(listc *l);

#ifdef __cplusplus
}
#endif

#endif
