.. _ina226:

INA226 Bidirectional Power/Current Monitor
##########################################

Overview
********

Note: This page is referred from the file "zephyr/samples/sensor/ina219/README.rst".

This sample application measures bus voltage, shunt voltage, power and current,
every 2 seconds and prints them to console.
The calibration/configuration parameters can be set in the devicetree file.

References
**********

 - `INA226 sensor <https://www.ti.com/product/INA226>`_

Wiring
******

The supply voltage of the INA226 can be in the 2.7V to 5.5V range.
The common mode voltage of the measured bus can be in the 0V to 36V range.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ina226
   :board: mr_canhubk3
   :goals: build flash

Sample Output
=============
When monitoring a 12.0 V bus with a 500 Micro-Ohm shunt resistor
you should get a similar output as below, repeated every 2 seconds:

.. code-block:: console

        Bus Voltage: 11.965000 [V] , Shunt Voltage: 0.000210 [V] , Power: 5.125000 [W] , Current: 0.420000 [A]

A negative sign indicates current flowing in reverse direction:

.. code-block:: console

        Bus Voltage: 11.965000 [V] , Shunt Voltage: 0.000210 [V] , Power: 5.125000 [W] , Current: -0.420000 [A]
