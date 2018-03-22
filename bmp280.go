package bsbmp

import (
	"log"

	i2c "github.com/d2r2/go-i2c"
)

// BMP280 specific type
type BMP280 struct {
	// Unique calibration coefficients
	t1 uint16
	t2 int16
	t3 int16
	p1 uint16
	p2 int16
	p3 int16
	p4 int16
	p5 int16
	p6 int16
	p7 int16
	p8 int16
	p9 int16
	// Logger
	log *log.Logger
	// Enable verbose output
	debug bool
}

// Static cast to verify that type implement interface.
var _ SensorInterface = &BMP280{}

func (this *BMP280) debugf(format string, args ...interface{}) {
	if this.debug {
		lg := this.log
		lg.Printf("[bmp] DEBUG "+format, args...)
	}
}

func (this *BMP280) SetDebug(debug bool) {
	this.debug = debug
}

// Read compensation coefficients, unique for each sensor.
func (this *BMP280) ReadCoefficients(i2c *i2c.I2C) error {
	var err error
	buf, _, err := i2c.ReadRegBytes(BMP280_COEF_START, BMP280_COEF_COUNT*2)
	if err != nil {
		return err
	}
	this.t1 = getU16LE(buf[BMP280_COEF_T1_LSB_MSB-BMP280_COEF_START : BMP280_COEF_T1_LSB_MSB-BMP280_COEF_START+2])
	this.t2 = getS16LE(buf[BMP280_COEF_T2_LSB_MSB-BMP280_COEF_START : BMP280_COEF_T2_LSB_MSB-BMP280_COEF_START+2])
	this.t3 = getS16LE(buf[BMP280_COEF_T3_LSB_MSB-BMP280_COEF_START : BMP280_COEF_T3_LSB_MSB-BMP280_COEF_START+2])
	this.p1 = getU16LE(buf[BMP280_COEF_P1_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P1_LSB_MSB-BMP280_COEF_START+2])
	this.p2 = getS16LE(buf[BMP280_COEF_P2_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P2_LSB_MSB-BMP280_COEF_START+2])
	this.p3 = getS16LE(buf[BMP280_COEF_P3_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P3_LSB_MSB-BMP280_COEF_START+2])
	this.p4 = getS16LE(buf[BMP280_COEF_P4_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P4_LSB_MSB-BMP280_COEF_START+2])
	this.p5 = getS16LE(buf[BMP280_COEF_P5_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P5_LSB_MSB-BMP280_COEF_START+2])
	this.p6 = getS16LE(buf[BMP280_COEF_P6_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P6_LSB_MSB-BMP280_COEF_START+2])
	this.p7 = getS16LE(buf[BMP280_COEF_P7_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P7_LSB_MSB-BMP280_COEF_START+2])
	this.p8 = getS16LE(buf[BMP280_COEF_P8_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P8_LSB_MSB-BMP280_COEF_START+2])
	this.p9 = getS16LE(buf[BMP280_COEF_P9_LSB_MSB-BMP280_COEF_START : BMP280_COEF_P9_LSB_MSB-BMP280_COEF_START+2])
	return nil
}

func (this *BMP280) IsValidCoefficients() error {
	err := checkCoefficient(this.t1, "dig_T1")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.t2), "dig_T2")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.t3), "dig_T3")
	if err != nil {
		return err
	}
	err = checkCoefficient(this.p1, "dig_P1")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p2), "dig_P2")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p3), "dig_P3")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p4), "dig_P4")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p5), "dig_P5")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p6), "dig_P6")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p7), "dig_P7")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p8), "dig_P8")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.p9), "dig_P9")
	if err != nil {
		return err
	}
	return nil
}

func (this *BMP280) GetSensorSignature() uint8 {
	var signature byte = 0x58
	return signature
}

// Read register 0xF4 for "busy" flag, according to sensor specification.
func (this *BMP280) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP280_STATUS_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x8
	this.debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

func (this *BMP280) getOversamplingRation(accuracy AccuracyMode) byte {
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

// Read uncompensated temprature from sensor.
func (this *BMP280) readUncompTemp(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osr := this.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP280_CNTR_MEAS_REG, power|(osr<<5))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(this, i2c)
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

// Read and calculate temrature in C (celsius).
func (this *BMP280) ReadTemperatureMult100C(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	ut, err := this.readUncompTemp(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	err = this.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var1 := ((ut>>3 - int32(this.t1)<<1) * int32(this.t2)) >> 11
	this.debugf("var1=%v", var1)
	var2 := (((ut>>4 - int32(this.t1)) * (ut>>4 - int32(this.t1))) >> 12 * int32(this.t3)) >> 14
	this.debugf("var1=%v", var2)
	t_fine := var1 + var2
	this.debugf("t_fine=%v", t_fine)
	t := (t_fine*5 + 128) >> 8
	return t, nil
}

// Read atmospheric uncompensated pressure from sensor.
func (this *BMP280) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	var power byte = 1 // Forced mode
	osr := this.getOversamplingRation(accuracy)
	err := i2c.WriteRegU8(BMP280_CNTR_MEAS_REG, power|(osr<<2))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(this, i2c)
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

// Read and calculate atmospheric pressure in Pa (Pascal).
func (this *BMP280) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	ut, err := this.readUncompTemp(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	this.debugf("ut=%v", ut)

	up, err := this.readUncompPressure(i2c, accuracy)
	if err != nil {
		return 0, err
	}
	this.debugf("up=%v", up)

	err = this.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}

	var01 := ((ut>>3 - int32(this.t1)<<1) * int32(this.t2)) >> 11
	this.debugf("var01=%v", var01)
	var02 := (((ut>>4 - int32(this.t1)) * (ut>>4 - int32(this.t1))) >> 12 * int32(this.t3)) >> 14
	this.debugf("var01=%v", var02)
	t_fine := var01 + var02

	var1 := int64(t_fine) - 128000
	this.debugf("var1=%v", var1)
	var2 := var1 * var1 * int64(this.p6)
	this.debugf("var2=%v", var2)
	var2 += (var1 * int64(this.p5)) << 17
	var2 += int64(this.p4) << 35
	this.debugf("var2=%v", var2)
	var1 = (var1*var1*int64(this.p3))>>8 + (var1*int64(this.p2))<<12
	var1 = ((int64(1)<<47 + var1) * int64(this.p1)) >> 33
	this.debugf("var1=%v", var1)
	if var1 == 0 {
		return 0, nil
	}
	p1 := int64(1048576) - int64(up)
	p1 = ((p1<<31 - var2) * 3125) / var1
	var1 = (int64(this.p9) * (p1 >> 13) * (p1 >> 13)) >> 25
	var2 = (int64(this.p8) * p1) >> 19
	p1 = (p1+var1+var2)>>8 + int64(this.p7)<<4
	p2 := p1 * 10 / 256
	p := int32(p2)

	return p, nil
}
