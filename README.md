# SX1272 LoRa/LoRaWAN API for STM32
This is a project for the 2018-19 Summer Research Program at The University of Queensland. The aim of the research is to design and prototype an IoT Communication System and Stand-alone Power Supply for Transformer Condition Monitoring Application. Source codes can be found under Firmware directory and Altium files can be found under Hardware directory.

The design consists of 3 LoRa transmitters and 1 concentrator. Each node uses a STM32L073RZ Nucleo development board and a SX1272 shield from Semtech. Datasheets for these modules can be found under Documentations directory. The STM32L0 HAL drivers and hardware configurations are generated using CubeMX. CMSIS math libraries are also utilized for vibration signal processing. 

Current version of the API implements a class A LoRaWAN in star topology. 

# Stand-alone Power Supply Unit
The end-device is also equipped with one lithium-ion cell that can be charged by either photovoltaic panel or USB port.
