//--------------------------------------------------------------------------------------------------
//
// Copyright (c) 2018 Denis Dyakov
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

package bsbmp

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"

	i2c "github.com/d2r2/go-i2c"
)

// BMP180 sensors memory map
const (
	// BMP180 general registers
	BMP180_ID_REG        = 0xD0
	BMP180_CNTR_MEAS_REG = 0xF4
	BMP180_RESET         = 0xE0
	// BMP180 specific compensation register's block
	BMP180_COEF_START = 0xAA
	BMP180_COEF_BYTES = 22
	// BMP180 specific 3-byte reading out temprature and preassure
	BMP180_OUT_MSB_LSB_XLSB = 0xF6
)

// Unique BMP180 calibration coefficients
type CoeffBMP180 struct {
	// Registers storing unique calibration coefficients
	COEF_AA uint8
	COEF_AB uint8
	COEF_AC uint8
	COEF_AD uint8
	COEF_AE uint8
	COEF_AF uint8
	COEF_B0 uint8
	COEF_B1 uint8
	COEF_B2 uint8
	COEF_B3 uint8
	COEF_B4 uint8
	COEF_B5 uint8
	COEF_B6 uint8
	COEF_B7 uint8
	COEF_B8 uint8
	COEF_B9 uint8
	COEF_BA uint8
	COEF_BB uint8
	COEF_BC uint8
	COEF_BD uint8
	COEF_BE uint8
	COEF_BF uint8
}

func (v *CoeffBMP180) dig_AC1() int16 {
	return int16(uint16(v.COEF_AA)<<8 | uint16(v.COEF_AB))
}

func (v *CoeffBMP180) dig_AC2() int16 {
	return int16(uint16(v.COEF_AC)<<8 | uint16(v.COEF_AD))
}

func (v *CoeffBMP180) dig_AC3() int16 {
	return int16(uint16(v.COEF_AE)<<8 | uint16(v.COEF_AF))
}

func (v *CoeffBMP180) dig_AC4() uint16 {
	return uint16(v.COEF_B0)<<8 | uint16(v.COEF_B1)
}

func (v *CoeffBMP180) dig_AC5() uint16 {
	return uint16(v.COEF_B2)<<8 | uint16(v.COEF_B3)
}

func (v *CoeffBMP180) dig_AC6() uint16 {
	return uint16(v.COEF_B4)<<8 | uint16(v.COEF_B5)
}

func (v *CoeffBMP180) dig_B1() int16 {
	return int16(uint16(v.COEF_B6)<<8 | uint16(v.COEF_B7))
}

func (v *CoeffBMP180) dig_B2() int16 {
	return int16(uint16(v.COEF_B8)<<8 | uint16(v.COEF_B9))
}

func (v *CoeffBMP180) dig_MB() int16 {
	return int16(uint16(v.COEF_BA)<<8 | uint16(v.COEF_BB))
}

func (v *CoeffBMP180) dig_MC() int16 {
	return int16(uint16(v.COEF_BC)<<8 | uint16(v.COEF_BD))
}

func (v *CoeffBMP180) dig_MD() int16 {
	return int16(uint16(v.COEF_BE)<<8 | uint16(v.COEF_BF))
}

// SensorBMP180 specific type
type SensorBMP180 struct {
	Coeff *CoeffBMP180
}

// Static cast to verify at compile time
// that type implement interface.
var _ SensorInterface = &SensorBMP180{}

// ReadSensorID reads sensor signature. It may be used for validation,
// that proper code settings used for sensor data decoding.
func (v *SensorBMP180) ReadSensorID(i2c *i2c.I2C) (uint8, error) {
	id, err := i2c.ReadRegU8(BMP180_ID_REG)
	if err != nil {
		return 0, err
	}
	return id, nil
}

// ReadCoefficients reads compensation coefficients, unique for each sensor.
func (v *SensorBMP180) ReadCoefficients(i2c *i2c.I2C) error {
	_, err := i2c.WriteBytes([]byte{BMP180_COEF_START})
	if err != nil {
		return err
	}
	var coef1 [BMP180_COEF_BYTES]byte
	err = readDataToStruct(i2c, BMP180_COEF_BYTES,
		binary.LittleEndian, &coef1)
	if err != nil {
		return err
	}
	buf := bytes.NewBuffer(coef1[:])
	coeff := &CoeffBMP180{}
	err = binary.Read(buf, binary.LittleEndian, coeff)
	if err != nil {
		return err
	}
	v.Coeff = coeff
	return nil
}

// IsValidCoefficients verify that compensate registers
// are not empty, and thus are valid.
func (v *SensorBMP180) IsValidCoefficients() error {
	if v.Coeff != nil {
		err := checkCoefficient(uint16(v.Coeff.dig_AC1()), "AC1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_AC2()), "AC2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_AC3()), "AC3")
		if err != nil {
			return err
		}
		err = checkCoefficient(v.Coeff.dig_AC4(), "AC4")
		if err != nil {
			return err
		}
		err = checkCoefficient(v.Coeff.dig_AC5(), "AC5")
		if err != nil {
			return err
		}
		err = checkCoefficient(v.Coeff.dig_AC6(), "AC6")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_B1()), "B1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_B2()), "B2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_MB()), "MB")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_MC()), "MC")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_MD()), "MD")
		if err != nil {
			return err
		}
	} else {
		err := errors.New("CoeffBMP180 struct does not built")
		return err
	}
	return nil
}

// RecognizeSignature returns description of signature if it valid,
// otherwise - error.
func (v *SensorBMP180) RecognizeSignature(signature uint8) (string, error) {
	switch signature {
	case 0x55:
		return "BMP180", nil
	default:
		return "", errors.New(fmt.Sprintf("signature 0x%x doesn't belong to BMP180 series", signature))
	}
}

// IsBusy reads register 0xF4 for "busy" flag,
// according to sensor specification.
func (v *SensorBMP180) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP180_CNTR_MEAS_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x20
	lg.Debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

// readUncompTemp reads uncompensated temprature from sensor.
func (v *SensorBMP180) readUncompTemp(i2c *i2c.I2C) (int32, error) {
	err := i2c.WriteRegU8(BMP180_CNTR_MEAS_REG, 0x2F)
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	w, err := i2c.ReadRegU16BE(BMP180_OUT_MSB_LSB_XLSB)
	if err != nil {
		return 0, err
	}
	return int32(w), nil
}

func (v *SensorBMP180) getOversamplingRation(accuracy AccuracyMode) byte {
	var b byte
	switch accuracy {
	case ACCURACY_LOW, ACCURACY_ULTRA_LOW:
		b = 0
	case ACCURACY_STANDARD:
		b = 1
	case ACCURACY_HIGH:
		b = 2
	case ACCURACY_ULTRA_HIGH:
		b = 3
	default:
		// assign accuracy to lowest resolution by default
		b = 0
	}
	return b
}

// readUncompPressure reads atmospheric uncompensated pressure from sensor.
func (v *SensorBMP180) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	oss := v.getOversamplingRation(accuracy)
	lg.Debugf("oss=%v", oss)
	err := i2c.WriteRegU8(BMP180_CNTR_MEAS_REG, 0x34+(oss<<6))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP180_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	up := (int32(buf[0])<<16 + int32(buf[1])<<8 + int32(buf[2])) >> (8 - oss)
	return up, nil
}

// ReadTemperatureMult100C reads and calculates temprature in C (celsius) multiplied by 100.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP180) ReadTemperatureMult100C(i2c *i2c.I2C, mode AccuracyMode) (int32, error) {
	ut, err := v.readUncompTemp(i2c)
	if err != nil {
		return 0, err
	}
	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}
	// Calculate temperature according to sensor specification
	x1 := ((ut - int32(v.Coeff.dig_AC6())) * int32(v.Coeff.dig_AC5())) >> 15
	lg.Debugf("x1=%v", x1)
	x2 := (int32(v.Coeff.dig_MC()) << 11) / (x1 + int32(v.Coeff.dig_MD()))
	lg.Debugf("x2=%v", x2)
	b5 := x1 + x2
	lg.Debugf("b5=%v", b5)
	t := ((b5 + 8) >> 4) * 10
	lg.Debugf("t=%v", t)
	return t, nil
}

// ReadPressureMult10Pa reads and calculates atmospheric pressure in Pa (Pascal) multiplied by 10.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP180) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (uint32, error) {
	oss := v.getOversamplingRation(accuracy)
	ut, err := v.readUncompTemp(i2c)
	if err != nil {
		return 0, err
	}
	lg.Debugf("ut=%v", ut)

	up, err := v.readUncompPressure(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	lg.Debugf("up=%v", up)

	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	// Calculate pressure according to sensor specification
	x1 := ((ut - int32(v.Coeff.dig_AC6())) * int32(v.Coeff.dig_AC5())) >> 15
	lg.Debugf("x1=%v", x1)
	x2 := (int32(v.Coeff.dig_MC()) << 11) / (x1 + int32(v.Coeff.dig_MD()))
	lg.Debugf("x2=%v", x2)
	b5 := x1 + x2
	lg.Debugf("b5=%v", b5)
	b6 := b5 - 4000
	lg.Debugf("b6=%v", b6)
	x1 = (int32(v.Coeff.dig_B2()) * ((b6 * b6) >> 12)) >> 11
	lg.Debugf("x1=%v", x1)
	x2 = (int32(v.Coeff.dig_AC2()) * b6) >> 11
	lg.Debugf("x2=%v", x2)
	x3 := x1 + x2
	lg.Debugf("x3=%v", x3)
	b3 := (((int32(v.Coeff.dig_AC1())*4 + x3) << uint32(oss)) + 2) / 4
	lg.Debugf("b3=%v", b3)
	x1 = (int32(v.Coeff.dig_AC3()) * b6) >> 13
	lg.Debugf("x1=%v", x1)
	x2 = ((int32(v.Coeff.dig_B1()) * (b6 * b6)) >> 12) >> 16
	lg.Debugf("x2=%v", x2)
	x3 = ((x1 + x2) + 2) >> 2
	lg.Debugf("x3=%v", x3)
	b4 := (uint32(v.Coeff.dig_AC4()) * uint32(x3+32768)) >> 15
	lg.Debugf("b4=%v", b4)
	b7 := (uint32(up) - uint32(b3)) * (50000 >> uint32(oss))
	lg.Debugf("b7=%v", b7)
	var p1 int32
	if b7 < 0x80000000 {
		p1 = int32((b7 * 2) / b4)
	} else {
		p1 = int32((b7 / b4) * 2)
	}
	lg.Debugf("p=%v", p1)
	x1 = (p1 >> 8) * (p1 >> 8)
	lg.Debugf("x1=%v", x1)
	x1 = (x1 * 3038) >> 16
	lg.Debugf("x1=%v", x1)
	x2 = (-7357 * p1) >> 16
	lg.Debugf("x2=%v", x2)
	p1 += (x1 + x2 + 3791) >> 4
	lg.Debugf("p=%v", p1)
	p := uint32(p1) * 10
	return p, nil
}

// ReadHumidityMultQ2210 does nothing. Humidity function is not applicable for BMP180.
func (v *SensorBMP180) ReadHumidityMultQ2210(i2c *i2c.I2C, accuracy AccuracyMode) (bool, uint32, error) {
	// Not supported
	return false, 0, nil
}
