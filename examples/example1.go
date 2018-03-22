package main

import (
	"log"

	"github.com/d2r2/go-bsbmp"
	"github.com/d2r2/go-i2c"
)

func main() {
	// Use i2cdetect utility to find device address at i2c-bus
	i2c, err := i2c.NewI2C(0x76, 1)
	// i2c, err := i2c.NewI2C(0x77, 1)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()
	// Uncomment to get verbose output
	// i2c.SetDebug(true)

	// sensor, err := bsbmp.NewBMP(bsbmp.BMP180_TYPE, i2c)
	sensor, err := bsbmp.NewBMP(bsbmp.BMP280_TYPE, i2c)
	if err != nil {
		log.Fatal(err)
	}
	// Uncomment to get verbose output
	// sensor.SetDebug(true)

	err = sensor.IsValidCoefficients()
	if err != nil {
		log.Fatal(err)
	}
	
	// ut, err := sensor.ReadUncompTemp()
	// if err != nil {
	// 	log.Fatal(err)
	// }
	// log.Printf("Raw temprature value = %v\n", ut)
	
	// Read temperature in celsius degree
	t, err := sensor.ReadTemperatureC(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Temprature = %v*C\n", t)
	
	// Read atmospheric pressure in pascal
	p, err := sensor.ReadPressurePa(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v Pa\n", p)
	
	// Read atmospheric pressure in mmHg
	p1, err := sensor.ReadPressureMmHg(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v mmHg\n", p1)
	
	// Read atmospheric altitude in meters
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Altitude = %v m\n", a)
}
