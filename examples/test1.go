package main

import (
	"log"

	"github.com/d2r2/go-bmp"
	"github.com/d2r2/go-i2c"
)

func main() {
	i2c, err := i2c.NewI2C(0x76, 1)
	// i2c, err := i2c.NewI2C(0x77, 1)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()
	// i2c.Debug = true

	// sensor, err := bsbmp.NewBMP(bsbmp.BMP180_TYPE, i2c)
	sensor, err := bsbmp.NewBMP(bsbmp.BMP280_TYPE, i2c)
	if err != nil {
		log.Fatal(err)
	}
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
	t, err := sensor.ReadTemperatureC(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Temprature = %v*C\n", t)
	p, err := sensor.ReadPressurePa(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v Pa\n", p)
	p1, err := sensor.ReadPressureMmHg(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Pressure = %v mmHg\n", p1)
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_LOW)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Altitude = %v m\n", a)
}
