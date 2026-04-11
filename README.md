# vsc_idf_ESP32_CompSHT85_CompNVM
ESP32 project, using VSCode IDF v6.0 with software components SHT85 and NVM module. 

## Material
- ESP32 WROOM (4MB, Micro USB)
- SHT85 (soldered on adapter to 2.54mm dupont pins)
- Breadbord, wires, cable

## Description
Implementing a sensor node to cyclically measure ambient temperature and humidity 
using a ESP32, SHT85 and NVM storage. 
Goal is to read temperature and humidity via I2C, buffer the values together with timestamp
and store a batch of values in non volatile memory. 
The non volatile memory (NVM) shall be implemented using FATFS and wear levelling. 
These features are third party, as well as I2C driver, operating system, hardware abstraction layer. 


## Software modules
### IDF modules required
- I2C Master
- FATFS
- Wear levelling
- Operating system 

### Custom modules
- SHT85 handler
- NVM interface
