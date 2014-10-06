#ifndef SENSORADDR_H_
#define SENSORADDR_H_

#define SENSORADDR_TSL2561_LOW		0x29    // TSL2561 #1 (bottomplate camera)
#define SENSORADDR_TSL2561_HIGH		0x39    // TSL2561 #2 (bottomplate slit)

#define SENSORADDR_TMP102_LL		0x48    // temp sensor TMP102 (payload #1)
#define SENSORADDR_TMP102_LH		0x49    // temp sensor TMP102 (payload #2)
#define SENSORADDR_TMP102_HL		0x4A    // temp sensor TMP102 (bottomplate #1)
#define SENSORADDR_TMP102_HH		0x4B    // temp sensor TMP102 (bottomplate #2)

#define SENSORADDR_MLX90614			0x51    // IR thermometer (bottomplate)

#define I2C_ADD_ASSV_1          0x04     // master arduino

#endif /* SENSORADDR_H_ */
