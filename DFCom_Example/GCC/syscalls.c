#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include "stm32f10x.h"

extern char _end;
extern char _estack;

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        while ((USART2->SR & USART_SR_TC) == 0) {
        }
        USART2->DR = (uint8_t)ptr[i];
    }
    return len;
}

caddr_t _sbrk(int incr)
{
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;
    if (heap_end + incr > &_estack) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

void _exit(int status)
{
    (void)status;
    while (1) {
    }
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}
