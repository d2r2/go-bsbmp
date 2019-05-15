//--------------------------------------------------------------------------------------------------
//
// Copyright (c) 2018 Denis Dyakov
//    poritons Copyright (c) 2019 Iron Heart Consulting, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//--------------------------------------------------------------------------------------------------

//  go-bsbmp package implements reading sensors values and providing compensating the readings, based on a table of coefficents stored in the device.
//   Sensors supported:
//     BMP180 - Abs Press, Temp. (Not recommeneded for new designs)
//     BMP280 - Abs Press, Tewp.
//     BME280 - ABs Press, Temp, Relative Humidity
//     BMP388 - Abs Press, Temp.
//   Note: the BMP300 device was never produced
package bsbmp

import (
	"math"

	"github.com/d2r2/go-i2c"
)

// SensorType identify which Bosch Sensortec
// temperature and pressure sensor is used.
// BMP180 and BMP280 are supported.
type SensorType int

// Implement Stringer interface.
func (v SensorType) String() string {
	if v == BMP180 {
		return "BMP180"
	} else if v == BMP280 {
		return "BMP280"
	} else if v == BME280 {
		return "BME280"
	} else if v == BMP388 {
		return "BMP388"
	} else {
		return "!!! unknown !!!"
	}
}

const (
	// Bosch Sensortec pressure and temperature sensor model BMP180.
	BMP180 SensorType = iota
	// Bosch Sensortec pressure and temperature sensor model BMP280.
	BMP280
	// Bosch Sensortec pressure, temperature and relative humidity sensor model BME280.
	BME280
	// Bosch Sensortec pressure and temperature sensor model BMP388.
	BMP388
)

// Accuracy mode for calculation of atmospheric pressure and temprature.
// Impact to value accuracy, calculation time frame and power consumption.
type AccuracyMode int

const (
	ACCURACY_ULTRA_LOW  AccuracyMode = iota // x1 sample
	ACCURACY_LOW                            // x2 samples
	ACCURACY_STANDARD                       // x4 samples
	ACCURACY_HIGH                           // x8 samples
	ACCURACY_ULTRA_HIGH                     // x16 samples
	ACCURACY_HIGHEST                        // x32 samples - added in BMP388
)

// Abstract BMPx sensor interface
// to control and gather data.
type SensorInterface interface {
	// ReadSensorID read sensor identifier unuque for each sensor type.
	ReadSensorID(i2c *i2c.I2C) (uint8, error)
	// ReadCoefficients read coefficient's block unique for each sensor.
	ReadCoefficients(i2c *i2c.I2C) error
	// IsValidCoefficients verify that coefficient values are not empty.
	IsValidCoefficients() error
	// Verify, that specific sensor can own signature identifier and
	// return text description of this specific id.
	RecognizeSignature(signature uint8) (string, error)
	// IsBusy check via status register that sensor ready for data exchange.
	IsBusy(i2c *i2c.I2C) (bool, error)
	// Divide by 10 to get float temperature value in celsius.
	ReadTemperatureMult100C(i2c *i2c.I2C, mode AccuracyMode) (temperature int32, erro error)
	// Divide by 10 to get float preasure value in pascal.
	ReadPressureMult10Pa(i2c *i2c.I2C, mode AccuracyMode) (pressure uint32, erro error)
	// Divide by 1024 to get float humidity value in range [0..100]%.
	ReadHumidityMultQ2210(i2c *i2c.I2C, mode AccuracyMode) (supported bool, humidity uint32, erro error)
}

// BMP represent both sensors BMP180 and BMP280
// implementing same approach to control and gather data.
type BMP struct {
	sensorType SensorType
	i2c        *i2c.I2C
	bmp        SensorInterface
}

// NewBMP creates new sensor object.
func NewBMP(sensorType SensorType, i2c *i2c.I2C) (*BMP, error) {
	v := &BMP{sensorType: sensorType, i2c: i2c}
	switch sensorType {
	case BMP180:
		v.bmp = &SensorBMP180{}
	case BMP280:
		v.bmp = &SensorBMP280{}
	case BME280:
		v.bmp = &SensorBME280{}
	case BMP388:
		v.bmp = &SensorBMP388{}
	}

	id, err := v.ReadSensorID()
	if err != nil {
		return nil, err
	}
	_, err = v.bmp.RecognizeSignature(id)
	if err != nil {
		return nil, err
	}
	err = v.bmp.ReadCoefficients(i2c)
	if err != nil {
		return nil, err
	}
	return v, nil
}

// ReadSensorID reads sensor signature. It may be used for validation,
// that proper code settings used for sensor data decoding.
func (v *BMP) ReadSensorID() (uint8, error) {
	id, err := v.bmp.ReadSensorID(v.i2c)
	return id, err
}

func (v *BMP) IsValidCoefficients() error {
	return v.bmp.IsValidCoefficients()
}

// ReadTemperatureMult100C reads and calculates temrature in C (celsius) multiplied by 100.
// Multiplication approach allow to keep result as integer amount.
func (v *BMP) ReadTemperatureMult100C(accuracy AccuracyMode) (int32, error) {
	t, err := v.bmp.ReadTemperatureMult100C(v.i2c, accuracy)
	return t, err
}

// ReadTemperatureC reads and calculates temrature in C (celsius).
func (v *BMP) ReadTemperatureC(accuracy AccuracyMode) (float32, error) {
	t, err := v.bmp.ReadTemperatureMult100C(v.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	return float32(t) / 100, nil
}

// ReadPressureMult10Pa reads and calculates atmospheric pressure in Pa (Pascal) multiplied by 10.
// Multiplication approach allow to keep result as integer amount.
func (v *BMP) ReadPressureMult10Pa(accuracy AccuracyMode) (uint32, error) {
	p, err := v.bmp.ReadPressureMult10Pa(v.i2c, accuracy)
	return p, err
}

// ReadPressurePa reads and calculates atmospheric pressure in Pa (Pascal).
func (v *BMP) ReadPressurePa(accuracy AccuracyMode) (float32, error) {
	p, err := v.bmp.ReadPressureMult10Pa(v.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	return float32(p) / 10, err
}

// ReadPressureMmHg reads and calculates atmospheric pressure in mmHg (millimeter of mercury).
func (v *BMP) ReadPressureMmHg(accuracy AccuracyMode) (float32, error) {
	p, err := v.bmp.ReadPressureMult10Pa(v.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	// Amount of Pa in 1 mmHg
	var mmHg float32 = 133.322
	// Round up to 2 decimals after point
	p2 := float32(int(float32(p)/mmHg*10)) / 100
	return p2, nil
}

// ReadHumidityRH reads and calculate humidity %RH.
func (v *BMP) ReadHumidityRH(accuracy AccuracyMode) (bool, float32, error) {
	supported, h, err := v.bmp.ReadHumidityMultQ2210(v.i2c, accuracy)
	if !supported {
		return supported, 0, nil
	}
	if err != nil {
		return supported, 0, err
	}
	h2 := float32(h) / 1024
	return supported, h2, nil
}

// ReadAltitude reads and calculates altitude above sea level, if we assume
// that pressure at sea level is equal to 101325 Pa.
func (v *BMP) ReadAltitude(accuracy AccuracyMode) (float32, error) {
	p, err := v.bmp.ReadPressureMult10Pa(v.i2c, accuracy)
	if err != nil {
		return 0, err
	}
	// Approximate atmospheric pressure at sea level in Pa
	p0 := 1013250.0
	a := 44330 * (1 - math.Pow(float64(p)/p0, 1/5.255))
	// Round up to 2 decimals after point
	a2 := float32(int(a*100)) / 100
	return a2, nil
}
