package main

import (
	"log"

	"github.com/d2r2/go-bmp"
	"github.com/d2r2/go-i2c"
)

func main() {
	i2c, err := i2c.NewI2C(0x77, 2)
	if err != nil {
		log.Fatal(err)
	}
	defer i2c.Close()
	w, err := i2c.ReadRegU16BE(0xB8)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v\n", w)
	/*	_, err = i2c.Write([]byte{0xB8})
		if err != nil {
			log.Fatal(err)
		}
		buf := make([]byte, 2)
		_, err = i2c.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		log.Printf("%v\n", buf)*/
	// buf := make([]byte, 2)
	w, err = i2c.ReadRegU16BE(0xAA)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v\n", w)
	/*
		_, err = i2c.Write([]byte{0xAC})
		if err != nil {
			log.Fatal(err)
		}
		_, err = i2c.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		log.Printf("%v\n", buf)
		_, err = i2c.Write([]byte{0xAA})
		if err != nil {
			log.Fatal(err)
		}
		_, err = i2c.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		log.Printf("%v\n", buf)*/
	b, err := i2c.ReadRegU8(0xD0)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("Id: 0x%0X\n", b)

	sensor := bmp.NewBMP(bmp.BMP180, i2c)
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
	p, err := sensor.ReadPressure(bmp.AM_ULTRA_LOW_POWER)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("%v\n", p)
}
