package main

import (
	"log"

	"github.com/d2r2/go-bmp"
	"github.com/d2r2/go-i2c"
)

func main() {
	i2c, err := i2c.NewI2C(0x77, 1)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()

	sensor, err := bmp.NewBMP(bmp.BMP180, i2c)
	if err != nil {
		log.Fatal(err)
	}
	err = sensor.IsValidCoefficients()
	if err != nil {
		log.Fatal(err)
	}
	ut, err := sensor.ReadUncompTemp()
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v\n", ut)
	t, err := sensor.ReadTemperature()
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v*C\n", t)
	p, err := sensor.ReadPressureMmHg(bmp.AM_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v mmHg\n", p)
	a, err := sensor.ReadAltitude(bmp.AM_STANDARD)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v m\n", a)
}
