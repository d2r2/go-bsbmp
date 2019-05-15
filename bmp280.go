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

// BMP280 sensors memory map
const (
	// BMP280 general registers
	BMP280_ID_REG        = 0xD0
	BMP280_STATUS_REG    = 0xF3
	BMP280_CNTR_MEAS_REG = 0xF4
	BMP280_CONFIG        = 0xF5 // TODO: support IIR filter settings
	BMP280_RESET         = 0xE0
	// BMP280 specific compensation register's block
	BMP280_COEF_START = 0x88
	BMP280_COEF_BYTES = 12 * 2
	// BMP280 specific 3-byte reading out temprature and preassure
	BMP280_PRESS_OUT_MSB_LSB_XLSB = 0xF7
	BMP280_TEMP_OUT_MSB_LSB_XLSB  = 0xFA
)

// Unique BMP280 calibration coefficients
type CoeffBMP280 struct {
	// Registers storing unique calibration coefficients
	COEF_88 uint8
	COEF_89 uint8
	COEF_8A uint8
	COEF_8B uint8
	COEF_8C uint8
	COEF_8D uint8
	COEF_8E uint8
	COEF_8F uint8
	COEF_90 uint8
	COEF_91 uint8
	COEF_92 uint8
	COEF_93 uint8
	COEF_94 uint8
	COEF_95 uint8
	COEF_96 uint8
	COEF_97 uint8
	COEF_98 uint8
	COEF_99 uint8
	COEF_9A uint8
	COEF_9B uint8
	COEF_9C uint8
	COEF_9D uint8
	COEF_9E uint8
	COEF_9F uint8
}

func (v *CoeffBMP280) dig_T1() uint16 {
	return uint16(v.COEF_89)<<8 | uint16(v.COEF_88)
}

func (v *CoeffBMP280) dig_T2() int16 {
	return int16(uint16(v.COEF_8B)<<8 | uint16(v.COEF_8A))
}

func (v *CoeffBMP280) dig_T3() int16 {
	return int16(uint16(v.COEF_8D)<<8 | uint16(v.COEF_8C))
}

func (v *CoeffBMP280) dig_P1() uint16 {
	return uint16(v.COEF_8F)<<8 | uint16(v.COEF_8E)
}

func (v *CoeffBMP280) dig_P2() int16 {
	return int16(uint16(v.COEF_91)<<8 | uint16(v.COEF_90))
}

func (v *CoeffBMP280) dig_P3() int16 {
	return int16(uint16(v.COEF_93)<<8 | uint16(v.COEF_92))
}

func (v *CoeffBMP280) dig_P4() int16 {
	return int16(uint16(v.COEF_95)<<8 | uint16(v.COEF_94))
}

func (v *CoeffBMP280) dig_P5() int16 {
	return int16(uint16(v.COEF_97)<<8 | uint16(v.COEF_96))
}

func (v *CoeffBMP280) dig_P6() int16 {
	return int16(uint16(v.COEF_99)<<8 | uint16(v.COEF_98))
}

func (v *CoeffBMP280) dig_P7() int16 {
	return int16(uint16(v.COEF_9B)<<8 | uint16(v.COEF_9A))
}

func (v *CoeffBMP280) dig_P8() int16 {
	return int16(uint16(v.COEF_9D)<<8 | uint16(v.COEF_9C))
}

func (v *CoeffBMP280) dig_P9() int16 {
	return int16(uint16(v.COEF_9F)<<8 | uint16(v.COEF_9E))
}

// SensorBMP280 specific type
type SensorBMP280 struct {
	Coeff *CoeffBMP280
}

// Static cast to verify at compile time
// that type implement interface.
var _ SensorInterface = &SensorBMP280{}

// ReadSensorID reads sensor signature. It may be used for validation,
// that proper code settings used for sensor data decoding.
func (v *SensorBMP280) ReadSensorID(i2c *i2c.I2C) (uint8, error) {
	id, err := i2c.ReadRegU8(BMP280_ID_REG)
	if err != nil {
		return 0, err
	}
	return id, nil
}

// ReadCoefficients reads compensation coefficients, unique for each sensor.
func (v *SensorBMP280) ReadCoefficients(i2c *i2c.I2C) error {
	_, err := i2c.WriteBytes([]byte{BMP280_COEF_START})
	if err != nil {
		return err
	}
	var coef1 [BMP280_COEF_BYTES]byte
	err = readDataToStruct(i2c, BMP280_COEF_BYTES,
		binary.LittleEndian, &coef1)
	if err != nil {
		return err
	}
	buf := bytes.NewBuffer(coef1[:])
	coeff := &CoeffBMP280{}
	err = binary.Read(buf, binary.LittleEndian, coeff)
	if err != nil {
		return err
	}
	v.Coeff = coeff
	return nil
}

// IsValidCoefficients verify that compensate registers
// are not empty, and thus are valid.
func (v *SensorBMP280) IsValidCoefficients() error {
	if v.Coeff != nil {
		err := checkCoefficient(v.Coeff.dig_T1(), "dig_T1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_T2()), "dig_T2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_T3()), "dig_T3")
		if err != nil {
			return err
		}
		err = checkCoefficient(v.Coeff.dig_P1(), "dig_P1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P2()), "dig_P2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P3()), "dig_P3")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P4()), "dig_P4")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P5()), "dig_P5")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P6()), "dig_P6")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P7()), "dig_P7")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P8()), "dig_P8")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.dig_P9()), "dig_P9")
		if err != nil {
			return err
		}
	} else {
		err := errors.New("CoeffBMP280 struct does not build")
		return err
	}
	return nil
}

// RecognizeSignature returns description of signature if it valid,
// otherwise - error.
func (v *SensorBMP280) RecognizeSignature(signature uint8) (string, error) {
	switch signature {
	case 0x58:
		return "BMP280", nil
	case 0x56, 0x57:
		return "BMP280 (sample)", nil
	default:
		return "", errors.New(fmt.Sprintf("signature 0x%x doesn't belong to BMP280 series", signature))
	}
}

// IsBusy reads register 0xF3 for "busy" flag,
// according to sensor specification.
func (v *SensorBMP280) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP280_STATUS_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x8
	lg.Debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

func (v *SensorBMP280) getOversamplingRation(accuracy AccuracyMode) byte {
	var b byte
	switch accuracy {
	case ACCURACY_ULTRA_LOW:
		b = 1
	case ACCURACY_LOW:
		b = 2
	case ACCURACY_STANDARD:
		b = 3
	case ACCURACY_HIGH:
		b = 4
	case ACCURACY_ULTRA_HIGH:
		b = 5
	default:
		// assign accuracy to lowest resolution by default
		b = 1
	}
	return b
}

// readUncompTemprature reads uncompensated temprature from sensor.
func (v *SensorBMP280) readUncompTemprature(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP280_CNTR_MEAS_REG, power|(osrt<<5))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP280_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, nil
}

// readUncompPressure reads atmospheric uncompensated pressure from sensor.
func (v *SensorBMP280) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrp := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP280_CNTR_MEAS_REG, power|(osrp<<2))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP280_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return up, nil
}

// readUncompTempratureAndPressure reads temprature and
// atmospheric uncompensated pressure from sensor.
// BMP280 allows to read temprature and pressure in one cycle,
// BMP180 - doesn't.
func (v *SensorBMP280) readUncompTempratureAndPressure(i2c *i2c.I2C,
	accuracy AccuracyMode) (temprature int32, pressure int32, err error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(ACCURACY_STANDARD)
	osrp := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BMP280_CNTR_MEAS_REG, power|(osrt<<5)|(osrp<<2))
	if err != nil {
		return 0, 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP280_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	buf, _, err = i2c.ReadRegBytes(BMP280_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, up, nil
}

// ReadTemperatureMult100C reads and calculates temrature in C (celsius) multiplied by 100.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP280) ReadTemperatureMult100C(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	ut, err := v.readUncompTemprature(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var1 := ((ut>>3 - int32(v.Coeff.dig_T1())<<1) * int32(v.Coeff.dig_T2())) >> 11
	lg.Debugf("var1=%v", var1)
	var2 := (((ut>>4 - int32(v.Coeff.dig_T1())) * (ut>>4 - int32(v.Coeff.dig_T1()))) >> 12 *
		int32(v.Coeff.dig_T3())) >> 14
	lg.Debugf("var1=%v", var2)
	tFine := var1 + var2
	lg.Debugf("t_fine=%v", tFine)
	t := (tFine*5 + 128) >> 8
	return t, nil
}

// ReadPressureMult10Pa reads and calculates atmospheric pressure in Pa (Pascal) multiplied by 10.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP280) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (uint32, error) {
	ut, up, err := v.readUncompTempratureAndPressure(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	lg.Debugf("ut=%v, up=%v", ut, up)

	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var01 := ((ut>>3 - int32(v.Coeff.dig_T1())<<1) * int32(v.Coeff.dig_T2())) >> 11
	lg.Debugf("var01=%v", var01)
	var02 := (((ut>>4 - int32(v.Coeff.dig_T1())) * (ut>>4 - int32(v.Coeff.dig_T1()))) >> 12 *
		int32(v.Coeff.dig_T3())) >> 14
	lg.Debugf("var01=%v", var02)
	tFine := var01 + var02

	var1 := int64(tFine) - 128000
	lg.Debugf("var1=%v", var1)
	var2 := var1 * var1 * int64(v.Coeff.dig_P6())
	lg.Debugf("var2=%v", var2)
	var2 += (var1 * int64(v.Coeff.dig_P5())) << 17
	var2 += int64(v.Coeff.dig_P4()) << 35
	lg.Debugf("var2=%v", var2)
	var1 = (var1*var1*int64(v.Coeff.dig_P3()))>>8 + (var1*int64(v.Coeff.dig_P2()))<<12
	var1 = ((int64(1)<<47 + var1) * int64(v.Coeff.dig_P1())) >> 33
	lg.Debugf("var1=%v", var1)
	if var1 == 0 {
		return 0, nil
	}
	p1 := int64(1048576) - int64(up)
	p1 = ((p1<<31 - var2) * 3125) / var1
	var1 = (int64(v.Coeff.dig_P9()) * (p1 >> 13) * (p1 >> 13)) >> 25
	var2 = (int64(v.Coeff.dig_P8()) * p1) >> 19
	p1 = (p1+var1+var2)>>8 + int64(v.Coeff.dig_P7())<<4
	p2 := p1 * 10 / 256
	p := uint32(p2)

	return p, nil
}

// ReadHumidityMultQ2210 does nothing. Humidity function is not applicable for BMP280.
func (v *SensorBMP280) ReadHumidityMultQ2210(i2c *i2c.I2C, accuracy AccuracyMode) (bool, uint32, error) {
	// Not supported
	return false, 0, nil
}
