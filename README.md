Bosch Sensortec BMP180, BMP280, BME280, BMP388 temperature, atmospheric pressure and humidity sensors
=============================================================================================

[![Build Status](https://travis-ci.org/d2r2/go-bsbmp.svg?branch=master)](https://travis-ci.org/d2r2/go-bsbmp)
[![Go Report Card](https://goreportcard.com/badge/github.com/d2r2/go-bsbmp)](https://goreportcard.com/report/github.com/d2r2/go-bsbmp)
[![GoDoc](https://godoc.org/github.com/d2r2/go-bsbmp?status.svg)](https://godoc.org/github.com/d2r2/go-bsbmp)
[![MIT License](http://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

BMP180 ([pdf reference](https://raw.github.com/d2r2/go-bsbmp/master/docs/BST-BMP180-DS000-09.pdf)), BMP280 ([pdf reference](https://raw.github.com/d2r2/go-bsbmp/master/docs/BST-BMP280-DS001-11.pdf)) and BME280 ([pdf reference](https://raw.github.com/d2r2/go-bsbmp/master/docs/BST-BME280_DS001-12.pdf)) are populare sensors among Arduino and Raspberry PI developers.
Sensors are compact and quite accurately measuring, working via i2c bus interface:
![image](https://raw.github.com/d2r2/go-bsbmp/master/docs/bmp180_bmp280_bme280_1.jpg)

BMP388 ([pdf reference](https://raw.github.com/d2r2/go-bsbmp/master/docs/BST-BMP388-DS001-11.pdf)) is the next generation of the BMP280.  Improved temperature coefficient, and the addition of a FIFO. Parameters measured are Temperature and Absolute Atmospheric Pressure.

Here is a library written in [Go programming language](https://golang.org/) for Raspberry PI and counterparts, which gives you in the output temperature and atmospheric pressure values (making all necessary i2c-bus interracting and values computing).

Golang usage
------------


```go
func main() {
	// Create new connection to i2c-bus on 1 line with address 0x76.
	// Use i2cdetect utility to find device address over the i2c-bus
	i2c, err := i2c.NewI2C(0x76, 1)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()
	// Uncomment next line to supress verbose output
	//logger.ChangePackageLogLevel("i2c", logger.InfoLevel)

	sensor, err := bsbmp.NewBMP(bsbmp.BMP280, i2c)
	if err != nil {
		log.Fatal(err)
	}
	// Uncomment next line to supress verbose output
	//logger.ChangePackageLogLevel("bsbmp", logger.InfoLevel)

	// Read temperature in celsius degree
	t, err := sensor.ReadTemperatureC(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Temprature = %v*C\n", t)
	// Read atmospheric pressure in pascal
	p, err := sensor.ReadPressurePa(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v Pa\n", p)
	// Read atmospheric pressure in mmHg
	p1, err := sensor.ReadPressureMmHg(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v mmHg\n", p1)
	// Read atmospheric altitude in meters above sea level, if we assume
	// that pressure at see level is equal to 101325 Pa.
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Altitude = %v m\n", a)
}
```


Getting help
------------

GoDoc [documentation](http://godoc.org/github.com/d2r2/go-bsbmp)

Installation
------------

```bash
$ go get -u github.com/d2r2/go-bsbmp
```

Troubleshoting
--------------

- *How to obtain fresh Golang installation to RPi device (either any RPi clone):*
If your RaspberryPI golang installation taken by default from repository is outdated, you may consider
to install actual golang mannualy from official Golang [site](https://golang.org/dl/). Download
tar.gz file containing armv6l in the name. Follow installation instructions.

- *How to enable I2C bus on RPi device:*
If you employ RaspberryPI, use raspi-config utility to activate i2c-bus on the OS level.
Go to "Interfaceing Options" menu, to active I2C bus.
Probably you will need to reboot to load i2c kernel module.
Finally you should have device like /dev/i2c-1 present in the system.

- *How to find I2C bus allocation and device address:*
Use i2cdetect utility in format "i2cdetect -y X", where X may vary from 0 to 5 or more,
to discover address occupied by peripheral device. To install utility you should run
`apt install i2c-tools` on debian-kind system. `i2cdetect -y 1` sample output:
	```
	     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
	00:          -- -- -- -- -- -- -- -- -- -- -- -- --
	10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	70: -- -- -- -- -- -- 76 --    
	```

Contribute authors
------------------

* [Kevin Rowett](https://github.com/K6TD): new sensor BMP388 support implementation.


Contact
-------

Please use [Github issue tracker](https://github.com/d2r2/go-bsbmp/issues) for filing bugs or feature requests.


License
-------

Go-bsbmp is licensed under MIT License.
