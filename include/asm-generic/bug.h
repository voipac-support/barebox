#ifndef _ASM_GENERIC_BUG_H
#define _ASM_GENERIC_BUG_H

#include <linux/compiler.h>

#define BUG() do { \
	printf("BUG: failure at %s:%d/%s()!\n", __FILE__, __LINE__, __FUNCTION__); \
	panic("BUG!"); \
} while (0)
#define BUG_ON(condition) do { if (unlikely((condition)!=0)) BUG(); } while(0)


#define __WARN() do { 								\
	printf("WARNING: at %s:%d/%s()!\n", __FILE__, __LINE__, __FUNCTION__);	\
} while (0)

#ifndef WARN_ON
#define WARN_ON(condition) ({						\
	int __ret_warn_on = !!(condition);				\
	if (unlikely(__ret_warn_on))					\
		__WARN();						\
	unlikely(__ret_warn_on);					\
})
#endif

#ifndef WARN
#define WARN(condition, format...) ({					\
	int __ret_warn_on = !!(condition);				\
	if (unlikely(__ret_warn_on)) {					\
		__WARN();						\
		puts("WARNING: ");					\
		printf(format);						\
	}								\
	unlikely(__ret_warn_on);					\
})
#endif
#endif
