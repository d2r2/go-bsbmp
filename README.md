Bosch Sensortec BMP180, BMP280 temperature and atmospheric pressure sensors
===========================================================================

[![Build Status](https://travis-ci.org/d2r2/go-bsbmp.svg?branch=master)](https://travis-ci.org/d2r2/go-bsbmp)
[![Go Report Card](https://goreportcard.com/badge/github.com/d2r2/go-bsbmp)](https://goreportcard.com/report/github.com/d2r2/go-bsbmp)
[![GoDoc](https://godoc.org/github.com/d2r2/go-bsbmp?status.svg)](https://godoc.org/github.com/d2r2/go-bsbmp)
[![MIT License](http://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

BMP180 and BMP280 are populare sensors among Arduino and Raspberry PI developers.
Both sensors are small and quite accurate working via i2c bus interface: (photos)

Here is a library written in [Go programming language](https://golang.org/) for Raspberry PI and counterparts, which gives you in the output temperature and atmospheric pressure values (making all necessary i2c-bus interracting and values computnig).

Golang usage
------------


```go
func main() {
	// Use i2cdetect utility to find device address at i2c-bus
	i2c, err := i2c.NewI2C(0x76, 1)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()
	// Uncomment to seee verbose output
	// i2c.SetDebug(true)
	sensor, err := bsbmp.NewBMP(bsbmp.BMP280_TYPE, i2c)
	if err != nil {
		log.Fatal(err)
	}
	// Uncomment to seee verbose output
	// sensor.SetDebug(true)

	// Read temperature in celcius degree
	t, err := sensor.ReadTemperatureC(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Temprature = %v*C\n", t)
	// Read atmosphere pressure in pascal
	p, err := sensor.ReadPressurePa(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v Pa\n", p)
	// Read atmosphere pressure in mmHg
	p1, err := sensor.ReadPressureMmHg(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v mmHg\n", p1)
	// Read atmosphere altitude in meters
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Altitude = %v m\n", a)
}
```


Getting help
------------

GoDoc [documentation](http://godoc.org/github.com/d2r2/go-hd44780)

Installation
------------

```bash
$ go get -u github.com/d2r2/go-bsbmp
```

Troubleshoting
--------------

- If you employ RaspberryPI, use raspi-config utility to activate i2c-bus on the OS level.
Go to "Interfaceing Options" menu, to active I2C bus. Restart will require.
- Use i2cdetect utility in format "i2cdetect -y X", where X may vary from 0 to 5 or more,
to discover address occupied by device. To install utility you should run
`apt install i2c-tools` on debian-kind system. `i2detect -y 1` sample output:
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


License
-------

Go-dht is licensed under MIT License.
