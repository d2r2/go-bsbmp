package bsbmp

import (
	"log"

	i2c "github.com/d2r2/go-i2c"
)

// BMP180 specific type
type BMP180 struct {
	// Unique calibration coefficients
	ac1 int16
	ac2 int16
	ac3 int16
	ac4 uint16
	ac5 uint16
	ac6 uint16
	b1  int16
	b2  int16
	mb  int16
	mc  int16
	md  int16
	// Logger
	log *log.Logger
	// Enable verbose output
	debug bool
}

// Static cast to verify that type implement interface.
var _ SensorInterface = &BMP180{}

func (this *BMP180) debugf(format string, args ...interface{}) {
	if this.debug {
		lg := this.log
		lg.Printf("[bmp] DEBUG "+format, args...)
	}
}

func (this *BMP180) SetDebug(debug bool) {
	this.debug = debug
}

// Read compensation coefficients, unique for each sensor.
func (this *BMP180) ReadCoefficients(i2c *i2c.I2C) error {
	var err error
	buf, _, err := i2c.ReadRegBytes(BMP180_COEF_START, BMP180_COEF_COUNT*2)
	if err != nil {
		return err
	}
	this.ac1 = getS16BE(buf[BMP180_COEF_AC1_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC1_MSB_LSB-BMP180_COEF_START+2])
	this.ac2 = getS16BE(buf[BMP180_COEF_AC2_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC2_MSB_LSB-BMP180_COEF_START+2])
	this.ac3 = getS16BE(buf[BMP180_COEF_AC3_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC3_MSB_LSB-BMP180_COEF_START+2])
	this.ac4 = getU16BE(buf[BMP180_COEF_AC4_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC4_MSB_LSB-BMP180_COEF_START+2])
	this.ac5 = getU16BE(buf[BMP180_COEF_AC5_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC5_MSB_LSB-BMP180_COEF_START+2])
	this.ac6 = getU16BE(buf[BMP180_COEF_AC6_MSB_LSB-BMP180_COEF_START : BMP180_COEF_AC6_MSB_LSB-BMP180_COEF_START+2])
	this.b1 = getS16BE(buf[BMP180_COEF_B1_MSB_LSB-BMP180_COEF_START : BMP180_COEF_B1_MSB_LSB-BMP180_COEF_START+2])
	this.b2 = getS16BE(buf[BMP180_COEF_B2_MSB_LSB-BMP180_COEF_START : BMP180_COEF_B2_MSB_LSB-BMP180_COEF_START+2])
	this.mb = getS16BE(buf[BMP180_COEF_MB_MSB_LSB-BMP180_COEF_START : BMP180_COEF_MB_MSB_LSB-BMP180_COEF_START+2])
	this.mc = getS16BE(buf[BMP180_COEF_MC_MSB_LSB-BMP180_COEF_START : BMP180_COEF_MC_MSB_LSB-BMP180_COEF_START+2])
	this.md = getS16BE(buf[BMP180_COEF_MD_MSB_LSB-BMP180_COEF_START : BMP180_COEF_MD_MSB_LSB-BMP180_COEF_START+2])
	return nil
}

func (this *BMP180) IsValidCoefficients() error {
	err := checkCoefficient(uint16(this.ac1), "AC1")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.ac2), "AC2")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.ac3), "AC3")
	if err != nil {
		return err
	}
	err = checkCoefficient(this.ac4, "AC4")
	if err != nil {
		return err
	}
	err = checkCoefficient(this.ac5, "AC5")
	if err != nil {
		return err
	}
	err = checkCoefficient(this.ac6, "AC6")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.b1), "B1")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.b2), "B2")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.mb), "MB")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.mc), "MC")
	if err != nil {
		return err
	}
	err = checkCoefficient(uint16(this.md), "MD")
	if err != nil {
		return err
	}
	return nil
}

func (this *BMP180) GetSensorSignature() uint8 {
	var signature byte = 0x55
	return signature
}

// Read register 0xF4 for "busy" flag, according to sensor specification.
func (this *BMP180) IsBusy(i2c *i2c.I2C) (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := i2c.ReadRegU8(BMP180_CNTR_MEAS_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x20
	this.debugf("Busy flag=0x%0X", b)
	return b != 0, nil
}

// Read uncompensated temprature from sensor.
func (this *BMP180) readUncompTemp(i2c *i2c.I2C) (int32, error) {
	err := i2c.WriteRegU8(BMP180_CNTR_MEAS_REG, 0x2F)
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(this, i2c)
	if err != nil {
		return 0, err
	}
	w, err := i2c.ReadRegU16BE(BMP180_OUT_MSB_LSB_XLSB)
	if err != nil {
		return 0, err
	}
	return int32(w), nil
}

// Read and calculate temrature in C (celsius).
func (this *BMP180) ReadTemperatureMult100C(i2c *i2c.I2C, mode AccuracyMode) (int32, error) {
	ut, err := this.readUncompTemp(i2c)
	if err != nil {
		return 0, err
	}
	err = this.ReadCoefficients(i2c)
	if err != nil {
		return 0, err
	}
	// Calculate temperature according to sensor specification
	x1 := ((ut - int32(this.ac6)) * int32(this.ac5)) >> 15
	this.debugf("x1=%v", x1)
	x2 := (int32(this.mc) << 11) / (x1 + int32(this.md))
	this.debugf("x2=%v", x2)
	b5 := x1 + x2
	this.debugf("b5=%v", b5)
	t := ((b5 + 8) >> 4) * 10
	this.debugf("t=%v", t)
	return t, nil
}

func (this *BMP180) getOversamplingRation(accuracy AccuracyMode) byte {
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
	}
	return b
}

// Read atmospheric uncompensated pressure from sensor.
func (this *BMP180) readUncompPressure(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	oss := this.getOversamplingRation(accuracy)
	this.debugf("oss=%v", oss)
	err := i2c.WriteRegU8(BMP180_CNTR_MEAS_REG, 0x34+(oss<<6))
	if err != nil {
		return 0, err
	}
	_, err = waitForCompletion(this, i2c)
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

// Read and calculate atmospheric pressure in Pa (Pascal).
func (this *BMP180) ReadPressureMult10Pa(i2c *i2c.I2C, accuracy AccuracyMode) (int32, error) {
	oss := this.getOversamplingRation(accuracy)
	ut, err := this.readUncompTemp(i2c)
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

	// Calculate pressure according to sensor specification
	x1 := ((ut - int32(this.ac6)) * int32(this.ac5)) >> 15
	this.debugf("x1=%v", x1)
	x2 := (int32(this.mc) << 11) / (x1 + int32(this.md))
	this.debugf("x2=%v", x2)
	b5 := x1 + x2
	this.debugf("b5=%v", b5)
	b6 := b5 - 4000
	this.debugf("b6=%v", b6)
	x1 = (int32(this.b2) * ((b6 * b6) >> 12)) >> 11
	this.debugf("x1=%v", x1)
	x2 = (int32(this.ac2) * b6) >> 11
	this.debugf("x2=%v", x2)
	x3 := x1 + x2
	this.debugf("x3=%v", x3)
	b3 := (((int32(this.ac1)*4 + x3) << uint32(oss)) + 2) / 4
	this.debugf("b3=%v", b3)
	x1 = (int32(this.ac3) * b6) >> 13
	this.debugf("x1=%v", x1)
	x2 = ((int32(this.b1) * (b6 * b6)) >> 12) >> 16
	this.debugf("x2=%v", x2)
	x3 = ((x1 + x2) + 2) >> 2
	this.debugf("x3=%v", x3)
	b4 := (uint32(this.ac4) * uint32(x3+32768)) >> 15
	this.debugf("b4=%v", b4)
	b7 := (uint32(up) - uint32(b3)) * (50000 >> uint32(oss))
	this.debugf("b7=%v", b7)
	var p1 int32
	if b7 < 0x80000000 {
		p1 = int32((b7 * 2) / b4)
	} else {
		p1 = int32((b7 / b4) * 2)
	}
	this.debugf("p=%v", p1)
	x1 = (p1 >> 8) * (p1 >> 8)
	this.debugf("x1=%v", x1)
	x1 = (x1 * 3038) >> 16
	this.debugf("x1=%v", x1)
	x2 = (-7357 * p1) >> 16
	this.debugf("x2=%v", x2)
	p1 += (x1 + x2 + 3791) >> 4
	this.debugf("p=%v", p1)
	p := p1 * 10
	return p, nil
}
