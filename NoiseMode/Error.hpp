#ifndef ERROR_H
#define ERROR_H

void log_system_init();

void log_group(const char *label);

void info(const char *fmt, ...);

void warning(const char *fmt, ...);

void error(const char *fmt, ...);

void progress(float percentage);

#ifdef NDEBUG
#define Assert(expr) ((void)0)
#else
#define Assert(expr) \
	((expr) ? (void)0 : \
		error("Assertion \"%s\" failed in %s, line %d\n", \
				#expr, __FILE__, __LINE__))
#endif // NDEBUG

#endif // ERROR_H