# bme680-esp32-idf

## Introduction
This is an implementation of the BME680 API provided by Bosch Sensortec for the ESP32 in idf. Currently it supports all the basic functionality of the API via I2C. 

The original API can be found here:
https://github.com/BoschSensortec/BME680_driver
The precompiled library was downloaded from the Bosch website:
https://www.bosch-sensortec.com/bst/products/all_products/bsec

Nearly all of the implementation specific changes were made in bsec_iot_example.c, which includes the basic template for I2C communication with the sensor. The only other change was bsec_integration.c line 169 where BME680_I2C_ADDR_PRIMARY was changed to BME680_I2C_ADDR_SECONDARY. This was needed to get the library to work with my specific sensor, but is not something that you might need to do.

## Updating API
If you would like to update the API, or just download it yourself, you can download all you need from the Bosch website (https://www.bosch-sensortec.com/bst/products/all_products/bsec). Once you downloaded the package, here is where all the files you will need.

/API
    bme680.c
    bme680.h
    bme680_defs.h
/algo/bin/Normal_version/esp32
    bsec_datatypes.h
    bsec_interface.h
    libalgobsec.a
/example
    bsec_integration.c
    bsec_integration.h
    bsec_iot_example.c (blank template)

Take these files and replace the ones found the main and components folders in this project. 

## API Usage
For API usage refer to the original API repository at
https://github.com/BoschSensortec/BME680_driver