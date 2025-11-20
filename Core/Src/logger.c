#include "main.h"
#include "fatfs.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "gpio.h"

/*
 * logger.c
 *
 * Логування 8 логічних каналів на одному GPIO-порті з записом у CSV-файл
 * на SD-карту через FatFs. Записуються лише моменти зміни будь-якого входу.
 */

/* ------------------------------------------------------------------------- */
/* Конфігурація                                                              */
/* ------------------------------------------------------------------------- */

/* Кількість каналів */
#define LOGGER_NUM_CHANNELS      8U

/* Період опитування входів, мс */
#define LOGGER_POLL_PERIOD_MS    100U

/*
 * Усі входи підключені до одного порту. Тут задається цей порт
 * і номер молодшого біта, на якому починаються LOGGER_NUM_CHANNELS
 * послідовних входів.
 *
 * Приклад:
 *   LOGGER_GPIO_PORT = GPIOA
 *   LOGGER_FIRST_BIT = 0
 *   → канали розташовані на PA0..PA7.
 *
 * За потреби ці макроси слід змінити відповідно до фактичного призначення пінів.
 */
#define LOGGER_GPIO_PORT         Log_IN0_GPIO_Port
#define LOGGER_FIRST_BIT         0U

/* ------------------------------------------------------------------------- */
/* Внутрішні змінні                                                          */
/* ------------------------------------------------------------------------- */
/* Create SD Card object */
static FATFS SDFatFS;
/* Logical drive path for the SD card */
char SDPath[4] = "0:/";


/* Стан після дебаунсу */
static uint8_t  logger_state = 0U;
/* Лічильники для "вертикального" дебаунсу */
static uint8_t  logger_cnt0  = 0U;
static uint8_t  logger_cnt1  = 0U;
/* Попередній стабілізований стан (для виявлення змін) */
static uint8_t  logger_prev  = 0xFFU;
/* Час наступного опитування */
static uint32_t logger_next_poll_tick = 0U;

/* Робота з файлом через FatFs */
static FIL     logger_file;
static bool    logger_file_open = false;

/* ------------------------------------------------------------------------- */
/* Локальні функції                                                          */
/* ------------------------------------------------------------------------- */

/* Читання LOGGER_NUM_CHANNELS бітів з одного GPIO-порту */
static uint8_t Logger_ReadInputs(void)
{
    /* Читається регістр IDR один раз; далі виділяються LOGGER_NUM_CHANNELS
     * бітів, починаючи з LOGGER_FIRST_BIT.
     */
    uint32_t idr = LOGGER_GPIO_PORT->IDR;
    uint8_t value = (uint8_t)((idr >> LOGGER_FIRST_BIT) &
                              ((1U << LOGGER_NUM_CHANNELS) - 1U));
    return value;
}

/* "Вертикальний" дебаунс для 8 бітів одночасно */
static uint8_t Logger_Debounce8(uint8_t sample)
{
    uint8_t delta = (uint8_t)(sample ^ logger_state);

    logger_cnt1 ^= logger_cnt0;
    logger_cnt0  = (uint8_t)~logger_cnt0;
    logger_cnt0 &= delta;
    logger_cnt1 &= delta;

    uint8_t toggle = (uint8_t)(logger_cnt0 & logger_cnt1);
    logger_state ^= toggle;

    return logger_state;
}

/* Отримання часу у форматі "секунди, мілісекунди" від моменту старту */
static void Logger_GetTime(uint32_t *sec, uint16_t *ms)
{
    uint32_t t = HAL_GetTick();   /* мс від запуску */
    *sec = t / 1000U;
    *ms  = (uint16_t)(t % 1000U);
}

/* Запис одного рядка у CSV-файл */
static void Logger_WriteLine(uint8_t bits)
{
    if (!logger_file_open)
    {
        return;
    }

    uint32_t seconds;
    uint16_t millis;
    Logger_GetTime(&seconds, &millis);

    char buf[64];
    int len = snprintf(buf, sizeof(buf),
                       "%lu.%03u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                       (unsigned long)seconds, (unsigned int)millis,
                       (bits >> 0) & 1U,
                       (bits >> 1) & 1U,
                       (bits >> 2) & 1U,
                       (bits >> 3) & 1U,
                       (bits >> 4) & 1U,
                       (bits >> 5) & 1U,
                       (bits >> 6) & 1U,
                       (bits >> 7) & 1U);
    if (len <= 0)
    {
        return;
    }

    HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Indicate logging activity

    UINT bw;
    f_write(&logger_file, buf, (UINT)len, &bw);
    /* Примусова синхронізація мінімізує втрату даних при раптовому вимкненні */
    f_sync(&logger_file);

    HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Turn off indicator
}

/* ------------------------------------------------------------------------- */
/* Публічні функції                                                          */
/* ------------------------------------------------------------------------- */

/*
 * Ініціалізація логера.
 *
 *  - монтування файлової системи на SD-карті;
 *  - відкриття (або створення) файлу "log.csv" в режимі допису;
 *  - додавання заголовка, якщо файл тільки-но створено;
 *  - ініціалізація внутрішніх змінних дебаунсу й опитування.
 *
 * Повертає 0 при успіху, від'ємне значення у випадку помилки.
 */
int Logger_Init(void)
{
    FRESULT fr;

    logger_state           = 0U;
    logger_cnt0            = 0U;
    logger_cnt1            = 0U;
    logger_prev            = 0xFFU; /* щоб примусити перший запис */
    logger_next_poll_tick  = HAL_GetTick();
    logger_file_open       = false;

    /* Монтування файлової системи */
    fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);
    if (fr != FR_OK)
    {
        return -1;
    }

    /* Відкриття файлу в режимі допису (курсором в кінець) */
    fr = f_open(&logger_file, "/log.csv", FA_WRITE | FA_OPEN_ALWAYS);

    if (fr != FR_OK)
    {
        return -2;
    }

    logger_file_open = true;

    /* Якщо файл новий (розмір 0) – записати заголовок */
    if (f_size(&logger_file) == 0U)
    {
        const char header[] = "Time,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8\r\n";
        UINT bw;
        f_write(&logger_file, header, (UINT)strlen(header), &bw);
        f_sync(&logger_file);
    }

    return 0;
}

/*
 * Періодичний виклик алгоритму логування.
 *
 * Рекомендовано викликати цю функцію у головному циклі якомога частіше.
 * Усередині реалізовано власний таймер опитування з періодом
 * LOGGER_POLL_PERIOD_MS, тому зайвих викликів шкоди не завдають.
 */
void Logger_Task(void)
{
    if (!logger_file_open)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* Перевірка з урахуванням можливого переповнення лічильника */
    if ((int32_t)(now - logger_next_poll_tick) < 0)
    {
        return;
    }

    logger_next_poll_tick += LOGGER_POLL_PERIOD_MS;

    /* Зчитування "сирих" входів, дебаунс, виявлення змін */
    uint8_t sample    = Logger_ReadInputs();
    uint8_t debounced = Logger_Debounce8(sample);
    uint8_t delta     = (uint8_t)(debounced ^ logger_prev);

    if (delta != 0U)
    {
        logger_prev = debounced;
        Logger_WriteLine(debounced);
    }
}
