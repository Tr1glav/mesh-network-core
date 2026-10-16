// Дописывание в строку без выхода за буфер. Зачем оно — см. strbuf.h.

#include "strbuf.h"

#include <stdarg.h>
#include <stdio.h>

extern "C" size_t strAppend(char *buf, size_t cap, size_t at, const char *fmt, ...) {
    if (!buf || cap == 0)
        return 0;
    if (at >= cap)
        return cap - 1; // буфер уже полон: конец строки там, где он есть
    va_list ap;
    va_start(ap, fmt);
    const int wrote = vsnprintf(buf + at, cap - at, fmt, ap);
    va_end(ap);
    // Ошибка форматирования: vsnprintf мог оставить буфер в любом виде, завершаем сами.
    if (wrote < 0) {
        buf[at] = 0;
        return at;
    }
    const size_t room = cap - at - 1; // сколько влезло бы без места под завершающий ноль
    return at + ((size_t)wrote > room ? room : (size_t)wrote);
}
