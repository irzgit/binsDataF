#ifndef MEDIAN_H
#define MEDIAN_H

template < int SIZE, typename TYPE >
class GMedian {
public:
    TYPE filtered(TYPE newVal) {
        buffer[_count] = newVal;
        if ((_count < _numRead - 1) && (buffer[_count] > buffer[_count + 1])) {
            for (int i = _count; i < _numRead - 1; i++) {
                if (buffer[i] > buffer[i + 1]) {
                    TYPE buff = buffer[i];
                    buffer[i] = buffer[i + 1];
                    buffer[i + 1] = buff;
                }
            }
        } else {
            if ((_count > 0) and (buffer[_count - 1] > buffer[_count])) {
                for (int i = _count; i > 0; i--) {
                    if (buffer[i] < buffer[i - 1]) {
                        TYPE buff = buffer[i];
                        buffer[i] = buffer[i - 1];
                        buffer[i - 1] = buff;
                    }
                }
            }
        }
        if (++_count >= _numRead) _count = 0;
        return buffer[(int)_numRead / 2];
    }
private:
    TYPE buffer[SIZE];
    char _count = 0;
    char _numRead = SIZE;
};

#endif // MEDIAN_H
