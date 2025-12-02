
// моторые XY брезен+DDA , моторы A B чисто DDA ( меньше структуры и ссылок.


typedef struct {  // отдельно шаги, отдельно интервалы. флипфлоп все дела.
    unsigned int interval;
    unsigned int lastStepTime;
    int targetX, targetY; // обязательно знаковые, с безхзнаковыми не сработает.
    int currentX, currentY; // обязательно знаковые, с безхзнаковыми не сработает.
    int error;
    bool finished; // готовый рэди
} LineDrawer;



void initLineDrawer(LineDrawer* ld, int targetX, int targetY, unsigned int interval) {
    ld->interval = interval;
    ld->targetX = targetX;
    ld->targetY = targetY;
    ld->currentX = 0;
    ld->currentY = 0;
    ld->error = ld->targetX - ld->targetY;
    ld->finished = false;
}


int stepsByBresenham(LineDrawer* ld, int time) {
    // тут структурку по временис DDA надо. интервал с FPEA флипфлоп - задает интервал по длинной оси.
    // интервал короткой оси не нужон, а значит можно будет тормозить одним интервалом. при торможение помним оригинальную пару числе интервала
    // оперируем целой частью чтоб не заморчаиваться - дробную зануляем на торможение, потом возвращаем. если торможение не нужно.

    if ((ld->lastStepTime+ld->interval)>time) return 0;
    ld->lastStepTime = time;

    if (ld->finished) return 0;
    int stepMask = 0;

    if (ld->currentX == ld->targetX && ld->currentY == ld->targetY) {
        ld->finished = true;
        return 0;
    }

    int error2 = ld->error * 2;
    if (error2 > -ld->targetY) {
        ld->error -= ld->targetY;
        ld->currentX += 1;
        stepMask |= 0x01;
    }
    if (error2 < ld->targetX) {
        ld->error += ld->targetX;
        ld->currentY += 1;
        stepMask |= 0x02;
    }

    return stepMask;
}

