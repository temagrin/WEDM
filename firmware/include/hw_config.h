#define MOTOR_ENABLE_INVERT 1   

#define MOTOR_STEP_INVERT   0

#define MOTOR_X_DIR_INVERT  0   // 0 = Направление X по умолчанию
#define MOTOR_Y_DIR_INVERT  0   // 0 = Направление Y по умолчанию
#define MOTOR_A_DIR_INVERT  0   // 0 = Направление A по умолчанию
#define MOTOR_B_DIR_INVERT  0   // 0 = Направление B по умолчанию


// базовый пин сигналов степ... занимает 4ре следующихъ за ним пина 0, 1, 2, 3
#define STEP_BASE_PIN 0

// не используются в коде - чисто для наглядности. стейт машина использует базовый пин
#define STEP_X_PIN 0
#define STEP_Y_PIN 1
#define STEP_A_PIN 2
#define STEP_B_PIN 3

#define DIR_X_PIN 4
#define DIR_Y_PIN 5
#define DIR_A_PIN 8
#define DIR_B_PIN 9

// один общий энейбл для XY мотора, остальные индивидуальные
#define EN_X_Y_PIN 10
#define EN_A_PIN 11
#define EN_B_PIN 12


#define HX711SCK_PIN 14
#define HX711DT_PIN 15

#define LED1_PIN 20
#define LED2_PIN 22

#define PULSE_PIN 21

#define CURRENT_SENCE_ADC_PIN 26

#define SPI0_MISO_PIN 16
#define SPI0_MOSI_PIN 17
#define SPI0_SCK_PIN 18
#define SPI0_CS_PIN 19

#define EMPTY_0_PIN 6
#define EMPTY_1_PIN 7
