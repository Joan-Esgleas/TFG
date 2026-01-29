// See LICENSE for license details.

#ifndef __SYSCALLS_H
#define __SYSCALLS_H


#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>
#include <sys/signal.h>
#include "util.h"

#define SYS_write 64

#undef strcmp


static uintptr_t syscall(uintptr_t which, uint64_t arg0, uint64_t arg1, uint64_t arg2);

void setStats(int enable);

void __attribute__((noreturn)) tohost_exit(uintptr_t code);

uintptr_t __attribute__((weak)) handle_trap(uintptr_t cause, uintptr_t epc, uintptr_t regs[32]);

void exit(int code);

void abort();

void printstr(const char* s);

void __attribute__((weak)) thread_entry(int cid, int nc);

int __attribute__((weak)) main(int argc, char** argv);

static void init_tls();

static void init_hpm();
void _init(int cid, int nc);

#undef putchar
int putchar(int ch);

void printhex(uint64_t x);

static inline void printnum(void (*putch)(int, void**), void **putdat,
                    unsigned long long num, unsigned base, int width, int padc);

static unsigned long long getuint(va_list *ap, int lflag);

static long long getint(va_list *ap, int lflag);
static void vprintfmt(void (*putch)(int, void**), void **putdat, const char *fmt, va_list ap);
int printf(const char* fmt, ...);

int sprintf(char* str, const char* fmt, ...);

void* memcpy(void* dest, const void* src, size_t len);

void* memset(void* dest, int byte, size_t len);

size_t strlen(const char *s);

size_t strnlen(const char *s, size_t n);

int strcmp(const char* s1, const char* s2);

char* strcpy(char* dest, const char* src);

long atol(const char* str);

#endif //__SYSCALLS_H
