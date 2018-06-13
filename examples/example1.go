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
	defer logger.FinalizeLogger()
	// Create new connection to i2c-bus on 1 line with address 0x76.
	// Use i2cdetect utility to find device address over the i2c-bus
	i2c, err := i2c.NewI2C(0x77, 0)
	if err != nil {
		lg.Fatal(err)
	}
	defer i2c.Close()

	lg.Notify("***************************************************************************************************")
	lg.Notify("*** You can change verbosity of output, to modify logging level of modules \"i2c\", \"bsbmp\"")
	lg.Notify("*** Uncomment/comment corresponding lines with call to ChangePackageLogLevel(...)")
	lg.Notify("***************************************************************************************************")
	// Uncomment/comment next lines to suppress/increase verbosity of output
	logger.ChangePackageLogLevel("i2c", logger.InfoLevel)
	logger.ChangePackageLogLevel("bsbmp", logger.InfoLevel)

	// sensor, err := bsbmp.NewBMP(bsbmp.BMP180_TYPE, i2c)
	// sensor, err := bsbmp.NewBMP(bsbmp.BMP280_TYPE, i2c)
	sensor, err := bsbmp.NewBMP(bsbmp.BME280, i2c)
	if err != nil {
		lg.Fatal(err)
	}

	id, err := sensor.ReadSensorID()
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("This Bosch Sensortec sensor has signature: 0x%x", id)

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
	p, err = sensor.ReadPressureMmHg(bsbmp.ACCURACY_LOW)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Pressure = %v mmHg", p)

	// Read atmospheric pressure in mmHg
	supported, h1, err := sensor.ReadHumidityRH(bsbmp.ACCURACY_LOW)
	if supported {
		if err != nil {
			lg.Fatal(err)
		}
		lg.Infof("Humidity = %v %%", h1)
	}

	// Read atmospheric altitude in meters above sea level, if we assume
	// that pressure at see level is equal to 101325 Pa.
	a, err := sensor.ReadAltitude(bsbmp.ACCURACY_LOW)
	if err != nil {
		lg.Fatal(err)
	}
	lg.Infof("Altitude = %v m", a)
}
