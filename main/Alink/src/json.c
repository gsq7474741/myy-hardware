#include "json.h"

void flush_str(char *buf, void *priv)
{
	json_gen_test_result_t *result = (json_gen_test_result_t *) priv;
	if (result) {
		if (strlen(buf) > sizeof(result->buf) - result->offset) {
			printf("Result Buffer too small\r\n");
			return;
		}
		memcpy(result->buf + result->offset, buf, strlen(buf));
		result->offset += strlen(buf);
	}
}