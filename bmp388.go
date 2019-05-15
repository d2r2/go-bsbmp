//--------------------------------------------------------------------------------------------------
//
// Copyright (c) 2018 Denis Dyakov
//   Portions Copyright (c) 2019 Iron Heart Consulting, LLC
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
	BMP388_ID_REG     = 0x00
	BMP388_STATUS_REG = 0x03
	BMP388_ERR_REG    = 0x02
	//	BMP388_CNTR_MEAS_REG = 0xF4  // No such reg in BMP388
	BMP388_ODR_REG      = 0x1D // Data Rate control
	BMP388_OSR_REG      = 0x1D // Over sample rate control
	BMP388_PWR_CTRL_REG = 0x1B // enable/disable press or temp, set operating mode
	// CONFIG Register is used to set IIR Filter coefficent
	BMP388_CONFIG = 0x1F // TODO: support IIR filter settings
	//	BMP388_RESET         = 0xE0 // TODO: '388 doesn't have a reset register
	BMP388_CMD_REG = 0x7E
	//  cmds - nop, extmode, clear FIFO, softreset
	// BMP388 specific compensation register's block
	BMP388_COEF_START = 0x31
	BMP388_COEF_BYTES = 21
	// BMP388 specific 3-byte reading out temprature and preassure
	BMP388_PRES_OUT_MSB_LSB_XLSB = 0x04
	BMP388_TEMP_OUT_MSB_LSB_XLSB = 0x07

	BMP388_PWR_MODE_SLEEP  = 0
	BMP388_PWR_MODE_FORCED = 1
	BMP388_PWR_MODE_NORMAL = 3

	// IIR Filter coefficent
	BMP388_coef_0   = 0 // bypass-mode
	BMP388_coef_1   = 0
	BMP388_coef_3   = 0
	BMP388_coef_7   = 0
	BMP388_coef_15  = 0
	BMP388_coef_31  = 0
	BMP388_coef_63  = 0
	BMP388_coef_127 = 0
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

func (v *CoeffBMP388) PAR_T1() uint16 {
	return uint16(v.COEF_32)<<8 | uint16(v.COEF_31)
}

func (v *CoeffBMP388) PAR_T2() uint16 {
	return uint16(uint16(v.COEF_34)<<8 | uint16(v.COEF_33))
}

func (v *CoeffBMP388) PAR_T3() int8 {
	return int8(v.COEF_35)
}

func (v *CoeffBMP388) PAR_P1() int16 {
	return int16(uint16(v.COEF_37)<<8 | uint16(v.COEF_36))
}

func (v *CoeffBMP388) PAR_P2() int16 {
	return int16(uint16(v.COEF_39)<<8 | uint16(v.COEF_38))
}

func (v *CoeffBMP388) PAR_P3() int8 {
	return int8(uint16(v.COEF_3A))
}

func (v *CoeffBMP388) PAR_P4() int8 {
	return int8(uint16(v.COEF_3B))
}

func (v *CoeffBMP388) PAR_P5() uint16 {
	return uint16(uint16(v.COEF_3D)<<8 | uint16(v.COEF_3C))
}

func (v *CoeffBMP388) PAR_P6() uint16 {
	return uint16(uint16(v.COEF_3F)<<8 | uint16(v.COEF_3E))
}

func (v *CoeffBMP388) PAR_P7() int8 {
	return int8(uint16(v.COEF_40))
}

func (v *CoeffBMP388) PAR_P8() int8 {
	return int8(uint16(v.COEF_41))
}

func (v *CoeffBMP388) PAR_P9() int16 {
	return int16(uint16(v.COEF_43)<<8 | uint16(v.COEF_42))
}

func (v *CoeffBMP388) PAR_P10() int8 {
	return int8(uint16(v.COEF_44))
}

func (v *CoeffBMP388) PAR_P11() int8 {
	return int8(uint16(v.COEF_45))
}

// SensorBMP388 specific type
type SensorBMP388 struct {
	Coeff *CoeffBMP388
}

// Static cast to verify at compile time
// that type implement interface.
var _ SensorInterface = &SensorBMP388{}

// ReadSensorID reads sensor signature. It may be used for validation,
// that proper code settings used for sensor data decoding.
func (v *SensorBMP388) ReadSensorID(i2c *i2c.I2C) (uint8, error) {
	id, err := i2c.ReadRegU8(BMP388_ID_REG)
	if err != nil {
		return 0, err
	}
	return id, nil
}

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
	lg.Debugf("PAR_T1:%v", v.Coeff.PAR_T1())
	lg.Debugf("PAR_T2:%v", v.Coeff.PAR_T2())
	lg.Debugf("PAR_T3:%v", v.Coeff.PAR_T3())
	lg.Debugf("PAR_P1:%v", v.Coeff.PAR_P1())
	lg.Debugf("PAR_P2:%v", v.Coeff.PAR_P2())
	lg.Debugf("PAR_P3:%v", v.Coeff.PAR_P3())
	lg.Debugf("PAR_P4:%v", v.Coeff.PAR_P4())
	lg.Debugf("PAR_P5:%v", v.Coeff.PAR_P5())
	lg.Debugf("PAR_P6:%v", v.Coeff.PAR_P6())
	lg.Debugf("PAR_P7:%v", v.Coeff.PAR_P7())
	lg.Debugf("PAR_P8:%v", v.Coeff.PAR_P8())
	lg.Debugf("PAR_P9:%v", v.Coeff.PAR_P9())
	lg.Debugf("PAR_P10:%v", v.Coeff.PAR_P10())
	lg.Debugf("PAR_P11:%v", v.Coeff.PAR_P11())
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
//  BMP388 has three separate busy/done flags - pres, temp, and cmd
//  this routine is called by a 'waitFor Completion' shared by the other BMP parts, which all have a combined
//    busy/done bit.
//    for now - we return TRUE when any of the done bits go true
//   TODO: break out the busy polling
func (v *SensorBMP388) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP388_STATUS_REG)
	if err != nil {
		return false, err
	}
	lg.Debugf("Busy flag=0x%0X", b)
	b = b & 0x60 // ignore cmd done
	return b == 0, nil
}

func (v *SensorBMP388) getOversamplingRation(accuracy AccuracyMode) byte {
	var b byte
	switch accuracy {
	case ACCURACY_ULTRA_LOW:
		b = 0
	case ACCURACY_LOW:
		b = 1
	case ACCURACY_STANDARD:
		b = 2
	case ACCURACY_HIGH:
		b = 3
	case ACCURACY_ULTRA_HIGH:
		b = 4
	case ACCURACY_HIGHEST:
		b = 5
	default:
		// assign accuracy to lowest resolution by default
		b = 0
	}
	return b
}

// readUncompTemprature reads uncompensated temprature from sensor.
func (v *SensorBMP388) readUncompTemprature(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	//  set IIR filter to bypass
	err := i2c.WriteRegU8(BMP388_CONFIG, BMP388_coef_0<<1)
	if err != nil {
		return 0, err
	}
	//   set over sample rate to 1x
	osrt := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BMP388_OSR_REG, osrt<<3)
	if err != nil {
		return 0, err
	}
	// enable pres and temp measuremeent, start a measurment
	var power byte = (BMP388_PWR_MODE_FORCED << 4) | 3 // enable pres, temp, FORCED operating mode
	lg.Debugf("power=0x%0X", power)
	err = i2c.WriteRegU8(BMP388_PWR_CTRL_REG, power)
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
	ut := int32(uint32(buf[0]) + uint32(buf[1])<<8 + uint32(buf[2])<<16)
	return ut, nil
}

// readUncompPressure reads atmospheric uncompensated pressure from sensor.
func (v *SensorBMP388) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = (BMP388_PWR_MODE_FORCED << 4) | 3 // enable pres, temp, FORCED operating mode
	err := i2c.WriteRegU8(BMP388_PWR_CTRL_REG, power)
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	osrp := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BMP388_OSR_REG, osrp)
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, err
	}
	buf, _, err := i2c.ReadRegBytes(BMP388_PRES_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, err
	}
	up := int32(buf[0]) + int32(buf[1])<<8 + int32(buf[2])<<16
	return up, nil
}

// readUncompTempratureAndPressure reads temprature and
// atmospheric uncompensated pressure from sensor.
// BMP388 allows to read temprature and pressure in one cycle,
// BMP180 - doesn't.
func (v *SensorBMP388) readUncompTempratureAndPressure(i2c *i2c.I2C,
	accuracy AccuracyMode) (temprature int32, pressure int32, err error) {
	var power byte = (BMP388_PWR_MODE_FORCED << 4) | 3 // enable pres, temp, FORCED operating mode
	err = i2c.WriteRegU8(BMP388_PWR_CTRL_REG, power)
	if err != nil {
		return 0, 0, err
	}
	_, err = waitForCompletion(v, i2c)
	if err != nil {
		return 0, 0, err
	}
	osrt := v.getOversamplingRation(ACCURACY_STANDARD)
	osrp := v.getOversamplingRation(accuracy)
	err = i2c.WriteRegU8(BMP388_OSR_REG, (osrt<<3)|osrp)
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
	ut := int32(buf[0]) + int32(buf[1])<<8 + int32(buf[2])<<16
	buf, _, err = i2c.ReadRegBytes(BMP388_PRES_OUT_MSB_LSB_XLSB, 3)
	if err != nil {
		return 0, 0, err
	}
	up := int32(buf[0]) + int32(buf[1])<<8 + int32(buf[2])<<16
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

	//  comp formula - taken from BMP3 API on github
	partial_data1 := uint64(ut - int32(256*int32(v.Coeff.PAR_T1())))
	partial_data2 := uint64(v.Coeff.PAR_T2()) * partial_data1
	partial_data3 := partial_data1 * partial_data1
	partial_data4 := int64(partial_data3) * int64(v.Coeff.PAR_T3())
	partial_data5 := (int64(partial_data2*262144) + partial_data4)
	partial_data6 := partial_data5 / 4294967269
	t := int32(partial_data6 * 25 / 16384)
	lg.Debugf("ut=%v", ut)
	lg.Debugf("d1=%v ", partial_data1)
	lg.Debugf("p_d2=%v ", partial_data2)
	lg.Debugf("p_d3=%v ", partial_data3)
	lg.Debugf("p_d4=%v ", partial_data4)
	lg.Debugf("p_d5=%v ", partial_data5)
	lg.Debugf("p_d6=%v ", partial_data6)
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

	//  Comp temp for use in pressure comp
	//  comp formula - taken from BMP3 API on github
	partial_data1_t := uint64(ut - int32(256*int32(v.Coeff.PAR_T1())))
	partial_data2_t := uint64(v.Coeff.PAR_T2()) * partial_data1_t
	partial_data3_t := partial_data1_t * partial_data1_t
	partial_data4_t := int64(partial_data3_t) * int64(v.Coeff.PAR_T3())
	partial_data5_t := (int64(partial_data2_t*262144) + partial_data4_t)
	partial_data6_t := partial_data5_t / 4294967269
	t_lin := partial_data6_t
	lg.Debugf("t_lin=%v", t_lin)
	lg.Debugf("----------")

	//  Compensate pressure - fixed point/integer arthmetic
	//  taken form formulas written in github
	partial_data1 := t_lin * t_lin
	partial_data2 := partial_data1 / 64
	partial_data3 := (partial_data2 * t_lin) / 256
	partial_data4 := (int64(v.Coeff.PAR_P8()) * partial_data3) / 32
	partial_data5 := (int64(v.Coeff.PAR_P7()) * partial_data1) * 16
	partial_data6 := (int64(v.Coeff.PAR_P6()) * t_lin) * 4194304
	offset := (int64(v.Coeff.PAR_P5()) * 140737488355328) + partial_data4 + partial_data5 + partial_data6
	lg.Debugf("partial_data1=%v", partial_data1)
	lg.Debugf("partial_data2=%v", partial_data2)
	lg.Debugf("partial_data3=%v", partial_data3)
	lg.Debugf("partial_data4=%v", partial_data4)
	lg.Debugf("partial_data5=%v", partial_data5)
	lg.Debugf("partial_data6=%v", partial_data6)
	lg.Debugf("offset=%v", offset)
	lg.Debugf("----------")

	partial_data2 = (int64(v.Coeff.PAR_P4()) * partial_data3) / 32
	partial_data4 = (int64(v.Coeff.PAR_P3()) * partial_data1) * 4
	partial_data5 = (int64(v.Coeff.PAR_P2()) - 16384) * t_lin * 2097152
	sensitivity := ((int64(v.Coeff.PAR_P1()) - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5
	lg.Debugf("partial_data2=%v", partial_data2)
	lg.Debugf("partial_data4=%v", partial_data4)
	lg.Debugf("partial_data5=%v", partial_data5)
	lg.Debugf("sensitivity=%v", sensitivity)
	lg.Debugf("----------")

	partial_data1 = (sensitivity / 16777216) * int64(up)
	partial_data2 = int64(v.Coeff.PAR_P10()) * t_lin
	partial_data3 = partial_data2 + (65536 * int64(v.Coeff.PAR_P9()))
	partial_data4 = (partial_data3 * int64(up)) / 8192
	partial_data5 = (partial_data4 * int64(up)) / 512
	partial_data6 = int64(uint64(up) * uint64(up))
	lg.Debugf("----------")
	lg.Debugf("partial_data1=%v", partial_data1)
	lg.Debugf("partial_data2=%v", partial_data2)
	lg.Debugf("partial_data3=%v", partial_data3)
	lg.Debugf("partial_data4=%v", partial_data4)
	lg.Debugf("partial_data5=%v", partial_data5)
	lg.Debugf("partial_data6=%v", partial_data6)
	lg.Debugf("----------")
	partial_data2 = (int64(v.Coeff.PAR_P11()) * partial_data6) / 65536
	partial_data3 = (partial_data2 * int64(up)) / 128
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3
	lg.Debugf("partial_data2=%v", partial_data2)
	lg.Debugf("partial_data3=%v", partial_data3)
	lg.Debugf("partial_data4=%v", partial_data4)
	comp_press := uint32((uint64(partial_data4) * 25) / 1099511627776)

	return comp_press, nil
}

// ReadHumidityMultQ2210 does nothing. Humidity function is not applicable for BMP388.
func (v *SensorBMP388) ReadHumidityMultQ2210(i2c *i2c.I2C, accuracy AccuracyMode) (bool, uint32, error) {
	// Not supported
	return false, 0, nil
}
