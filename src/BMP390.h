#include <stdint.h>
#include <math.h>
#ifndef TG_BMP_390_H
#define TG_BMP_390_H


struct calibData {
   uint16_t PAR_T1   = 0;
   uint16_t PAR_T2   = 0;
   int8_t PAR_T3     = 0;

   int16_t PAR_P1    = 0;
   int16_t PAR_P2    = 0;
   int8_t PAR_P3     = 0;
   int8_t PAR_P4     = 0;
   uint16_t PAR_P5   = 0;
   uint16_t PAR_P6   = 0;
   int8_t PAR_P7     = 0;
   int8_t PAR_P8     = 0;
   int16_t PAR_P9    = 0;
   int8_t PAR_P10    = 0;
   int8_t PAR_P11    = 0;

   int64_t linTemp;

   calibData(uint8_t* nvm_par, uint8_t arrLen) {
      if(arrLen<21)
         return;

      PAR_T1  = (((uint16_t)nvm_par[1] << 8) | (uint16_t)nvm_par[0]);
      PAR_T2  = (((uint16_t)nvm_par[3] << 8) | (uint16_t)nvm_par[2]);
      PAR_T3  = (int8_t)nvm_par[4];

      PAR_P1  = (int16_t)(((uint16_t)nvm_par[6] << 8) | (uint16_t)nvm_par[5]);
      PAR_P2  = (int16_t)(((uint16_t)nvm_par[8] << 8) | (uint16_t)nvm_par[7]);
      PAR_P3  = (int8_t)nvm_par[9];
      PAR_P4  = (int8_t)nvm_par[10];
      PAR_P5  = (((uint16_t)nvm_par[12] << 8) | (uint16_t)nvm_par[11]);
      PAR_P6  = (((uint16_t)nvm_par[14] << 8) | (uint16_t)nvm_par[13]);
      PAR_P7  = (int8_t)nvm_par[15];
      PAR_P8  = (int8_t)nvm_par[16];
      PAR_P9  = (int16_t)(((uint16_t)nvm_par[18] << 8) | (uint16_t)nvm_par[17]);
      PAR_P10 = (int8_t)nvm_par[19];
      PAR_P11 = (int8_t)nvm_par[20];
   }
} ;

class BMP390 {
      private:
         static inline const uint8_t DEFAULT_PWR_SETTINGS      = 0b00100011;
                                                                 //76543210
         static inline const uint8_t DEFAULT_OSR_SETTINGS      = 0b00000000;
         static inline const uint8_t DEFAULT_ODR_SETTINGS      = 0x00;
         static inline const uint8_t DEFAULT_CONFIG_SETTINGS   = 0b00000000;

         static inline uint8_t BMP_ADDR;

         static inline calibData* compData = nullptr;

         uint8_t NVM_PAR[21] = {0};

         bool I2Cwrite(uint8_t addr, uint8_t* data, uint8_t len);

         bool I2Cread(uint8_t addr, uint8_t* data, uint8_t len);

         bool getCompensationData();

         uint8_t readRaw();

         static inline uint8_t rawData[6];
         static inline uint32_t rawTemp;
         static inline uint64_t rawPres;

      public:

         BMP390(bool addrLSB = 0);

         uint8_t begin();
         
         void getCalibData(float* array);
         bool setSettings(uint8_t PWR_Settings, uint8_t ODR_Settings, uint8_t OSR_Settings, uint8_t CONFIG_Settings);
         
         void softReset();

         int64_t calcTemp(int64_t raw, calibData* parData);
         uint64_t calcPres(int64_t raw, calibData* parData);

         uint8_t getStatus();

         void requestNewData();
         uint8_t getData(float *array, uint8_t arrLen);
};

#endif /* BME280_H_ */