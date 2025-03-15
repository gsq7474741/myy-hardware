#ifndef _JSON_H_
#define _JSON_H_

#include "json_generator.h"
#include "main.h"

typedef struct {
	char   buf[1024];
	size_t offset;
} json_gen_test_result_t;

void flush_str(char *buf, void *priv);

#endif





