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

// BMP388 sensors memory map
const (
	// BMP388 general registers
	BMP388_STATUS_REG    = 0x03
	BMP388_CNTR_MEAS_REG = 0xF4
	BMP388_CONFIG        = 0x1F // TODO: support IIR filter settings
	BMP388_RESET         = 0xE0 // TODO: '388 doesn't have a reset regsiter
	// BMP388 specific compensation register's block
	BMP388_COEF_START = 0x31
	BMP388_COEF_BYTES = 21
	// BMP388 specific 3-byte reading out temprature and preassure
	BMP388_PRESS_OUT_MSB_LSB_XLSB = 0xF7
	BMP388_TEMP_OUT_MSB_LSB_XLSB  = 0xFA
)

// Unique BMP388 calibration coefficients
type CoeffBMP388 struct {
	// Registers storing unique calibration coefficients
	COEF_31 uint8
	COEF_32 uint8
	COEF_33 uint8
	COEF_34 uint8
	COEF_35 uint8
	COEF_36 uint8
	COEF_37 uint8
	COEF_38 uint8
	COEF_39 uint8
	COEF_3A uint8
	COEF_3B uint8
	COEF_3C uint8
	COEF_3D uint8
	COEF_3E uint8
	COEF_3F uint8
	COEF_40 uint8
	COEF_41 uint8
	COEF_42 uint8
	COEF_43 uint8
	COEF_44 uint8
	COEF_45 uint8
}

// coef naming uses the nomenclature out of the datasheet
func (v *CoeffBMP388) PAR_T1() uint16 {
	return uint16(v.COEF_32)<<8 | uint16(v.COEF_31)
}

func (v *CoeffBMP388) PAR_T2() uint16 {
	return uint16(uint16(v.COEF_34)<<8 | uint16(v.COEF_33))
}

func (v *CoeffBMP388) PAR_T3() int16 {
	return int16(v.COEF_35)
}

func (v *CoeffBMP388) PAR_P1() int16 {
	return int16(uint16(v.COEF_37)<<8 | uint16(v.COEF_36))
}

func (v *CoeffBMP388) PAR_P2() int16 {
	return int16(uint16(v.COEF_39)<<8 | uint16(v.COEF_38))
}

func (v *CoeffBMP388) PAR_P3() int16 {
	return int16(uint16(v.COEF_3A))
}

func (v *CoeffBMP388) PAR_P4() int16 {
	return int16(uint16(v.COEF_3B))
}

func (v *CoeffBMP388) PAR_P5() uint16 {
	return uint16(uint16(v.COEF_3D)<<8 | uint16(v.COEF_3C))
}

func (v *CoeffBMP388) PAR_P6() uint16 {
	return uint16(uint16(v.COEF_3F)<<8 | uint16(v.COEF_3E))
}

func (v *CoeffBMP388) PAR_P7() int16 {
	return int16(uint16(v.COEF_40))
}

func (v *CoeffBMP388) PAR_P8() int16 {
	return int16(uint16(v.COEF_41))
}

func (v *CoeffBMP388) PAR_P9() int16 {
	return int16(uint16(v.COEF_43)<<8 | uint16(v.COEF_42))
}

func (v *CoeffBMP388) PAR_P10() int16 {
	return int16(uint16(v.COEF_44))
}

func (v *CoeffBMP388) PAR_P11() int16 {
	return int16(uint16(v.COEF_45))
}

// SensorBMP388 specific type
type SensorBMP388 struct {
	Coeff *CoeffBMP388
}

// Static cast to verify at compile time
// that type implement interface.
var _ SensorInterface = &SensorBMP388{}

// ReadCoefficients reads compensation coefficients, unique for each sensor.
func (v *SensorBMP388) ReadCoefficients(i2c *i2c.I2C) error {
	_, err := i2c.WriteBytes([]byte{BMP388_COEF_START})
	if err != nil {
		return err
	}
	var coef1 [BMP388_COEF_BYTES]byte
	err = readDataToStruct(i2c, BMP388_COEF_BYTES,
		binary.LittleEndian, &coef1)
	if err != nil {
		return err
	}
	buf := bytes.NewBuffer(coef1[:])
	coeff := &CoeffBMP388{}
	err = binary.Read(buf, binary.LittleEndian, coeff)
	if err != nil {
		return err
	}
	v.Coeff = coeff
	return nil
}

// IsValidCoefficients verify that compensate registers
// are not empty, and thus are valid.
func (v *SensorBMP388) IsValidCoefficients() error {
// TODO:  research a better test for valid Coef.  Refeence code doesn't check
	if v.Coeff != nil {
		err := checkCoefficient(uint16(v.Coeff.PAR_T1()), "PAR_T1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_T2()), "PAR_T2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_T3()), "PAR_T3")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P1()), "PAR_P1")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P2()), "PAR_P2")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P3()), "PAR_P3")
		if err != nil {
			return err
		}
//		err = checkCoefficient(uint16(v.Coeff.PAR_P4()), "PAR_P4")
//		if err != nil {
//			return err
//		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P5()), "PAR_P5")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P6()), "PAR_P6")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P7()), "PAR_P7")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P8()), "PAR_P8")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P9()), "PAR_P9")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P10()), "PAR_P10")
		if err != nil {
			return err
		}
		err = checkCoefficient(uint16(v.Coeff.PAR_P11()), "PAR_P11")
		if err != nil {
			return err
		}
	} else {
		err := errors.New("CoeffBMP388 struct does not build")
		return err
	}
	return nil
}

// RecognizeSignature returns description of signature if it valid,
// otherwise - error.
func (v *SensorBMP388) RecognizeSignature(signature uint8) (string, error) {
	switch signature {
	case 0x50:
		return "BMP388", nil
	default:
		return "", errors.New(fmt.Sprintf("signature 0x%x doesn't belong to BMP388 series", signature))
	}
}

// IsBusy reads register 0xF3 for "busy" flag,
// according to sensor specification.
func (v *SensorBMP388) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP388_STATUS_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x8
	lg.Debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

func (v *SensorBMP388) getOversamplingRation(accuracy AccuracyMode) byte {
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
	}
	return b
}

// readUncompTemprature reads uncompensated temprature from sensor.
func (v *SensorBMP388) readUncompTemprature(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP388_CNTR_MEAS_REG, power|(osrt<<5))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP388_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, nil
}

// readUncompPressure reads atmospheric uncompensated pressure from sensor.
func (v *SensorBMP388) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osrp := v.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP388_CNTR_MEAS_REG, power|(osrp<<2))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP388_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return up, nil
}

// readUncompTempratureAndPressure reads temprature and
// atmospheric uncompensated pressure from sensor.
// BMP388 allows to read temprature and pressure in one cycle,
// BMP180 - doesn't.
func (v *SensorBMP388) readUncompTempratureAndPressure(i2c *i2c.I2C,
	accuracy AccuracyMode) (temprature int32, pressure int32, err error) {
	var power byte = 1 // Forced mode
	osrt := v.getOversamplingRation(ACCURACY_STANDARD)
	osrp := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BMP388_CNTR_MEAS_REG, power|(osrt<<5)|(osrp<<2))
	if err != nil {
		return 0, 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP388_TEMP_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	ut := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	buf, _, err = i2c.ReadRegBytes(BMP388_PRESS_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	up := int32(buf[0])<<12 + int32(buf[1])<<4 + int32(buf[2]&0xF0)>>4
	return ut, up, nil
}

// ReadTemperatureMult100C reads and calculates temrature in C (celsius) multiplied by 100.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP388) ReadTemperatureMult100C(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	ut, err := v.readUncompTemprature(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var1 := ((ut>>3 - int32(v.Coeff.PAR_T1())<<1) * int32(v.Coeff.PAR_T2())) >> 11
	lg.Debugf("var1=%v", var1)
	var2 := (((ut>>4 - int32(v.Coeff.PAR_T1())) * (ut>>4 - int32(v.Coeff.PAR_T1()))) >> 12 *
		int32(v.Coeff.PAR_T3())) >> 14
	lg.Debugf("var1=%v", var2)
	tFine := var1 + var2
	lg.Debugf("t_fine=%v", tFine)
	t := (tFine*5 + 128) >> 8
	return t, nil
}

// ReadPressureMult10Pa reads and calculates atmospheric pressure in Pa (Pascal) multiplied by 10.
// Multiplication approach allow to keep result as integer number.
func (v *SensorBMP388) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (uint32, error) {
	ut, up, err := v.readUncompTempratureAndPressure(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	lg.Debugf("ut=%v, up=%v", ut, up)

	err = v.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var01 := ((ut>>3 - int32(v.Coeff.PAR_T1())<<1) * int32(v.Coeff.PAR_T2())) >> 11
	lg.Debugf("var01=%v", var01)
	var02 := (((ut>>4 - int32(v.Coeff.PAR_T1())) * (ut>>4 - int32(v.Coeff.PAR_T1()))) >> 12 *
		int32(v.Coeff.PAR_T3())) >> 14
	lg.Debugf("var01=%v", var02)
	tFine := var01 + var02

	var1 := int64(tFine) - 128000
	lg.Debugf("var1=%v", var1)
	var2 := var1 * var1 * int64(v.Coeff.PAR_P6())
	lg.Debugf("var2=%v", var2)
	var2 += (var1 * int64(v.Coeff.PAR_P5())) << 17
	var2 += int64(v.Coeff.PAR_P4()) << 35
	lg.Debugf("var2=%v", var2)
	var1 = (var1*var1*int64(v.Coeff.PAR_P3()))>>8 + (var1*int64(v.Coeff.PAR_P2()))<<12
	var1 = ((int64(1)<<47 + var1) * int64(v.Coeff.PAR_P1())) >> 33
	lg.Debugf("var1=%v", var1)
	if var1 == 0 {
		return 0, nil
	}
	p1 := int64(1048576) - int64(up)
	p1 = ((p1<<31 - var2) * 3125) / var1
	var1 = (int64(v.Coeff.PAR_P9()) * (p1 >> 13) * (p1 >> 13)) >> 25
	var2 = (int64(v.Coeff.PAR_P8()) * p1) >> 19
	p1 = (p1+var1+var2)>>8 + int64(v.Coeff.PAR_P7())<<4
	p2 := p1 * 10 / 256
	p := uint32(p2)

	return p, nil
}

// ReadHumidityMultQ2210 does nothing. Humidity function is not applicable for BMP388.
func (v *SensorBMP388) ReadHumidityMultQ2210(i2c *i2c.I2C, accuracy AccuracyMode) (bool, uint32, error) {
	// Not supported
	return false, 0, nil
}
