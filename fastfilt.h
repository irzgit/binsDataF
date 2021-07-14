#ifndef FASTFILT_H
#define FASTFILT_H


// быстрый целочисленный экспоненциальный фильтр

#define FF_PASS_MAX 1
#define FF_PASS_MIN 2

class FastFilter {
public:
    // коэффициент 0-31
    FastFilter(char k = 20, int dt = 0) {
        setK(k);
        setDt(dt);
    }

    // коэффициент 0-31
    void setK(char k) {
        _k1 = k;
        _k2 = 32 - k;
    }

    // установить период фильтрации
    void setDt(int dt) {
        _dt = dt;
    }

    // установить режим пропуска (FF_PASS_MAX / FF_PASS_MIN)
    void setPass(char pass) {
        _pass = pass;
    }

    // установить исходное значение для фильтрации
    void setRaw(int raw) {
        _raw = raw;
    }

    // установить фильтрованное значение
    void setFil(int fil) {
        _raw_f = fil;
    }

    // проверка на переполнение
    bool checkPass(int val) {
        if (_pass == FF_PASS_MAX && val > _raw_f) {
            _raw_f = val;
            return 1;
        } else if (_pass == FF_PASS_MIN && val < _raw_f) {
            _raw_f = val;
            return 1;
        }
        return 0;
    }

    // расчёт по таймеру
//    void compute() {
//        if (_dt == 0 || millis() - _tmr >= _dt) {
//            _tmr = millis();
//            computeNow();
//        }
//    }

    // произвести расчёт сейчас
    void computeNow() {
        _raw_f = (_k1 * _raw_f + _k2 * _raw) >> 5;
    }

    // получить фильтрованное значение
    long getFil() {
        return _raw_f;
    }

    // получить последнее сырое значение
    long getRaw() {
        return _raw;
    }

private:
    int _tmr = 0;
    int _dt = 0;
    char _k1 = 20, _k2 = 12;
    char _pass = 0;
    int _raw_f = 0, _raw = 0;
};


#endif // FASTFILT_H
