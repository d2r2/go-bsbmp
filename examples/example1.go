package main

import (
	"github.com/d2r2/go-bsbmp"
	"github.com/d2r2/go-i2c"
	logger "github.com/d2r2/go-logger"
)

var lg = logger.NewPackageLogger("main",
	logger.DebugLevel,
	// logger.InfoLevel,
)

func main() {
	// Use i2cdetect utility to find device address over the i2c-bus
	i2c, err := i2c.NewI2C(0x76, 1)
	if err != nil {
		lg.Fatal(err)
	}
	defer i2c.Close()
	// Uncomment next line to supress verbose output
	// logger.ChangePackageLogLevel("i2c", logger.InfoLevel)

	// sensor, err := bsbmp.NewBMP(bsbmp.BMP180_TYPE, i2c)
	sensor, err := bsbmp.NewBMP(bsbmp.BMP280_TYPE, i2c)
	if err != nil {
		lg.Fatal(err)
	}
	// Uncomment next line to supress verbose output
	// logger.ChangePackageLogLevel("bsbmp", logger.InfoLevel)

	err = sensor.IsValidCoefficients()
	if err != nil {
		lg.Fatal(err)
	}

	// Read temperature in celsius degree
	t, err := sensor.ReadTemperatureC(bsbmp.ACCURACY_STANDARD)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Temprature = %v*C", t)

	// Read atmospheric pressure in pascal
	p, err := sensor.ReadPressurePa(bsbmp.ACCURACY_LOW)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Pressure = %v Pa", p)

	// Read atmospheric pressure in mmHg
	p1, err := sensor.ReadPressureMmHg(bsbmp.ACCURACY_LOW)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Pressure = %v mmHg", p1)

	// Read atmospheric altitude in meters
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_LOW)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Altitude = %v m", a)
}
