#ifndef RUNNINGAVERAGE_H
#define RUNNINGAVERAGE_H
#include "glwidget.h"

class GFilterRA {
public:
    GFilterRA();								// инициализация фильтра
    GFilterRA(float coef);						// расширенная инициализация фильтра (коэффициент)
    GFilterRA(float coef, int interval);	// расширенная инициализация фильтра (коэффициент, шаг фильтрации)
    void setCoef(float coef);	    			// настройка коэффициента фильтрации (0.00 - 1.00). Чем меньше, тем плавнее
    void setStep(int interval);			// установка шага фильтрации (мс). Чем меньше, тем резче фильтр
    void setCount(int value);

    float filteredTime(int value);			// возвращает фильтрованное значение с опорой на встроенный таймер
    float filtered(int value);				// возвращает фильтрованное значение

    float filteredTime(float value);			// возвращает фильтрованное значение с опорой на встроенный таймер
    float filtered(float value);				// возвращает фильтрованное значение
    float filteredTime(float value, int Timer);



private:
    float _coef = 0.0, _lastValue = 0.0;
    int _filterTimer = 0;
    int _filterInterval = 0;
    int count =0;
};



#endif // RUNNINGAVERAGE_H
