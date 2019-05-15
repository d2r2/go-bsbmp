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

// BME280 sensors memory map
const (
	// BME280 general registers
	BME280_ID_REG    = 0xD0
	BME280_CTRL_HUM  = 0xF2
	BME280_STATUS    = 0xF3
	BME280_CTRL_MEAS = 0xF4
	BME280_CONFIG    = 0xF5 // TODO: support IIR filter settings
	BME280_RESET     = 0xE0
	// BME280 specific compensation register's blocks
	BME280_COEF_PART1_START = 0x88
	BME280_COEF_PART1_BYTES = 12 * 2
	BME280_COEF_PART2_START = 0xA1
	BME280_COEF_PART2_BYTES = 1
	BME280_COEF_PART3_START = 0xE1
	BME280_COEF_PART3_BYTES = 1*2 + 5
	// BME280 specific 3-byte reading out temprature and preassure
	BME280_PRESS_OUT_MSB_LSB_XLSB = 0xF7
	BME280_TEMP_OUT_MSB_LSB_XLSB  = 0xFA
	BME280_HUM_OUT_MSB_LSB        = 0xFD
)

// Unique BME280 calibration coefficients
type CoeffBME280 struct {
	// Registers storing unique calibration coefficients.
	// Block 1
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
	// Block 2
	COEF_A1 uint8
	// Block 3
	COEF_E1 uint8
	COEF_E2 uint8
	COEF_E3 uint8
	COEF_E4 uint8
	COEF_E5 uint8
	COEF_E6 uint8
	COEF_E7 uint8
}

func (v *CoeffBME280) dig_T1() uint16 {
	return uint16(v.COEF_89)<<8 | uint16(v.COEF_88)
}

func (v *CoeffBME280) dig_T2() int16 {
	return int16(uint16(v.COEF_8B)<<8 | uint16(v.COEF_8A))
}

func (v *CoeffBME280) dig_T3() int16 {
	return int16(uint16(v.COEF_8D)<<8 | uint16(v.COEF_8C))
}

func (v *CoeffBME280) dig_P1() uint16 {
	return uint16(v.COEF_8F)<<8 | uint16(v.COEF_8E)
}

func (v *CoeffBME280) dig_P2() int16 {
	return int16(uint16(v.COEF_91)<<8 | uint16(v.COEF_90))
}

func (v *CoeffBME280) dig_P3() int16 {
	return int16(uint16(v.COEF_93)<<8 | uint16(v.COEF_92))
}

func (v *CoeffBME280) dig_P4() int16 {
	return int16(uint16(v.COEF_95)<<8 | uint16(v.COEF_94))
}

func (v *CoeffBME280) dig_P5() int16 {
	return int16(uint16(v.COEF_97)<<8 | uint16(v.COEF_96))
}

func (v *CoeffBME280) dig_P6() int16 {
	return int16(uint16(v.COEF_99)<<8 | uint16(v.COEF_98))
}

func (v *CoeffBME280) dig_P7() int16 {
	return int16(uint16(v.COEF_9B)<<8 | uint16(v.COEF_9A))
}

func (v *CoeffBME280) dig_P8() int16 {
	return int16(uint16(v.COEF_9D)<<8 | uint16(v.COEF_9C))
}

func (v *CoeffBME280) dig_P9() int16 {
	return int16(uint16(v.COEF_9F)<<8 | uint16(v.COEF_9E))
}

func (v *CoeffBME280) dig_H1() uint8 {
	return uint8(v.COEF_A1)
}

func (v *CoeffBME280) dig_H2() int16 {
	return int16(uint16(v.COEF_E2)<<8 | uint16(v.COEF_E1))
}

func (v *CoeffBME280) dig_H3() uint8 {
	return uint8(v.COEF_E3)
}

func (v *CoeffBME280) dig_H4() int16 {
	return int16(uint16(v.COEF_E4)<<4 | uint16(v.COEF_E5&0x0F))
}

func (v *CoeffBME280) dig_H5() int16 {
	return int16(uint16(v.COEF_E6)<<4 | uint16((v.COEF_E5>>4)&0x0F))
}

func (v *CoeffBME280) dig_H6() int8 {
	return int8(v.COEF_E7)
}

// SensorBME280 specific type
type SensorBME280 struct {
	Coeff *CoeffBME280
}

// Static cast to verify at compile time
// that type implement interface.
var _ SensorInterface = &SensorBME280{}

// ReadSensorID reads sensor signature. It may be used for validation,
// that proper code settings used for sensor data decoding.
func (v *SensorBME280) ReadSensorID(i2c *i2c.I2C) (uint8, error) {
	id, err := i2c.ReadRegU8(BME280_ID_REG)
	if err != nil {
		return 0, err
	}
	return id, nil
}

// ReadCoefficients reads compensation coefficients, unique for each sensor.
func (v *SensorBME280) ReadCoefficients(i2c *i2c.I2C) error {
	// read coefficients #1
	_, err := i2c.WriteBytes([]byte{BME280_COEF_PART1_START})
	if err != nil {
		return err
	}
	var coef1 [BME280_COEF_PART1_BYTES]byte
	err = readDataToStruct(i2c, BME280_COEF_PART1_BYTES,
		binary.LittleEndian, &coef1)
	if err != nil {
		return err
	}

	// read coefficients #2
	_, err = i2c.WriteBytes([]byte{BME280_COEF_PART2_START})
	if err != nil {
		return err
	}
	var coef2 [BME280_COEF_PART2_BYTES]byte
	err = readDataToStruct(i2c, BME280_COEF_PART2_BYTES,
		binary.LittleEndian, &coef2)
	if err != nil {
		return err
	}

	// read coefficients #3
	_, err = i2c.WriteBytes([]byte{BME280_COEF_PART3_START})
	if err != nil {
		return err
	}
	var coef3 [BME280_COEF_PART3_BYTES]byte
	err = readDataToStruct(i2c, BME280_COEF_PART3_BYTES,
		binary.LittleEndian, &coef3)
	if err != nil {
		return err
	}

	// combine coefficients altogether in single structure
	arr := coef1[:]
	arr = append(arr, coef2[:]...)
	arr = append(arr, coef3[:]...)
	buf := bytes.NewBuffer(arr)
	coeff := &CoeffBME280{}
	err = binary.Read(buf, binary.LittleEndian, coeff)
	if err != nil {
		return err
	}
	v.Coeff = coeff
	return nil
}

// IsValidCoefficients verify that compensate registers
// are not empty, and thus are valid.
func (v *SensorBME280) IsValidCoefficients() error {
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
		err := errors.New("CoeffBME280 struct does not build")
		return err
	}
	return nil
}

// RecognizeSignature returns description of signature if it valid,
// otherwise - error.
func (v *SensorBME280) RecognizeSignature(signature uint8) (string, error) {
	switch signature {
	case 0x60:
		return "BME280", nil
	default:
		return "", errors.New(fmt.Sprintf("signature 0x%x doesn't belong to BME280 series", signature))
	}
}

// IsBusy reads register 0xF3 for "busy" flag,
// according to sensor specification.
func (v *SensorBME280) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BME280_STATUS)
	if err != nil {
		return false, err
	}
	b = b & 0x8
	lg.Debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

func (v *SensorBME280) getOversamplingRation(accuracy AccuracyMode) byte {
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
func (v *SensorBME280) readUncompTemprature(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BME280_CTRL_MEAS, power|(osrt<<5))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BME280_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, nil
}

// readUncompPressure reads uncompensated atmospheric pressure from sensor.
func (v *SensorBME280) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrp := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BME280_CTRL_MEAS, power|(osrp<<2))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BME280_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return up, nil
}

// readUncompHumidity reads uncompensated humidity from sensor.
func (v *SensorBME280) readUncompHumidity(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BME280_CTRL_MEAS, power|(osrt<<5))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	osrh := v.getOversamplingRation(ACCURACY_ULTRA_LOW)
	err = i2c.WriteRegU8(BME280_CTRL_HUM, osrh)
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BME280_HUM_OUT_MSB_LSB, 2)
	if err != nil {
		return 0, err
	}
	uh := int32(buf[0])<<8 + int32(buf[1])
	return uh, nil
}

// readUncompTempratureAndPressure reads temprature and
// atmospheric uncompensated pressure from sensor.
// BME280 allows to read temprature and pressure in one cycle,
// BMP180 - doesn't.
func (v *SensorBME280) readUncompTempratureAndPressure(i2c *i2c.I2C,
	accuracy AccuracyMode) (temprature int32, pressure int32, err error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(ACCURACY_STANDARD)
	osrp := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BME280_CTRL_MEAS, power|(osrt<<5)|(osrp<<2))
	if err != nil {
		return 0, 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BME280_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	buf, _, err = i2c.ReadRegBytes(BME280_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, up, nil
}

// ReadTemperatureMult100C reads and calculates temrature in C (celsius) multiplied by 100.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBME280) ReadTemperatureMult100C(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
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
func (v *SensorBME280) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (uint32, error) {
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

// ReadHumidityMultQ2210 reads and calculate humidity in %RH.
// Multiplication approach allow to keep result as integer number.
// To get real value it's necessary to divide result by 1024.
func (v *SensorBME280) ReadHumidityMultQ2210(i2c *i2c.I2C,
	accuracy AccuracyMode) (supported bool, humidity uint32, erro error) {

	ut, err := v.readUncompTemprature(i2c, accuracy)
	if err != nil {
		return true, 0, err
	}
	uh, err := v.readUncompHumidity(i2c, accuracy)
	if err != nil {
		return true, 0, err
	}
	lg.Debugf("ut=%v, uh=%v", ut, uh)
	err = v.ReadCoefficients(i2c)
	if err != nil {
		return true, 0, err
	}

	var01 := ((ut>>3 - int32(v.Coeff.dig_T1())<<1) * int32(v.Coeff.dig_T2())) >> 11
	lg.Debugf("var01=%v", var01)
	var02 := (((ut>>4 - int32(v.Coeff.dig_T1())) * (ut>>4 - int32(v.Coeff.dig_T1()))) >> 12 *
		int32(v.Coeff.dig_T3())) >> 14
	lg.Debugf("var01=%v", var02)
	tFine := var01 + var02
	lg.Debugf("t_fine=%v", tFine)

	// Alternative version of humidity calculation from raw value
	// based on float ariphmetics.
	//
	// var var_H float64
	// var_H = float64(tFine) - 76800.0
	// var_H = (float64(uh) - ((float64(v.Coeff.dig_H4()))*64.0 + (float64(v.Coeff.dig_H5()))/16384.0*var_H)) *
	// 	(float64(v.Coeff.dig_H2()) / 65536.0 * (1.0 + (float64(v.Coeff.dig_H6()) / 67108864.0 * var_H *
	// 		(1.0 + (float64(v.Coeff.dig_H3()) / 67108864.0 * var_H)))))
	// var_H = var_H * (1.0 - (float64(v.Coeff.dig_H1()) * var_H / 524288.0))
	// if var_H > 100.0 {
	// 	var_H = 100.0
	// }
	// if var_H < 0.0 {
	// 	var_H = 0.0
	// }
	// return true, uint32(var_H * 1024), nil

	var v_x1 int32
	v_x1 = tFine - 76800
	lg.Debugf("v_x1=%v", v_x1)

	v_x1 = ((((uh << 14) - (int32(v.Coeff.dig_H4()) << 20) - (int32(v.Coeff.dig_H5()) * v_x1)) +
		16384) >> 15) * (((((((v_x1*int32(v.Coeff.dig_H6()))>>10)*(((v_x1*
		int32(v.Coeff.dig_H3()))>>11)+32768))>>10)+2097152)*
		int32(v.Coeff.dig_H2()) + 8192) >> 14)

	lg.Debugf("v_x1=%v", v_x1)

	v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * int32(v.Coeff.dig_H1())) >> 4)
	lg.Debugf("v_x1=%v", v_x1)

	if v_x1 < 0 {
		v_x1 = 0
	} else if v_x1 > 419430400 {
		v_x1 = 419430400
	}
	lg.Debugf("v_x1=%v", v_x1)
	v_x1 = v_x1 >> 12
	lg.Debugf("v_x1=%v", v_x1)
	return true, uint32(v_x1), nil
}
