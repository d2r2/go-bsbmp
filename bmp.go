package bmp

import (
	"time"

	"github.com/d2r2/go-i2c"
)

type SensorType int

// Implement Stringer interface.
func (this SensorType) String() string {
	if this == BMP085 {
		return "BMP085"
	} else if this == BMP180 {
		return "BMP180"
	} else {
		return "!!! unknown !!!"
	}
}

const (
	// Bosch pressure and temperature sensor model BMP085.
	BMP085 SensorType = iota
	// Bosch pressure and temperature sensor model BMP180.
	BMP180
)

type AccuracyMode int

const (
	AM_ULTRA_LOW_POWER AccuracyMode = iota
	AM_STANDARD
	AM_HIGH_RESOLUTION
	AM_ULTRA_HIGH_RESOLUTION
)

type BMP struct {
	sensorType SensorType
	i2c        *i2c.I2C
	// Calibration coefficients
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
}

func NewBMP(sensorType SensorType, i2c *i2c.I2C) *BMP {
	this := &BMP{sensorType: sensorType, i2c: i2c}
	return this
}

func (this *BMP) readCoefficients() error {
	var err error
	this.ac1, err = this.i2c.ReadRegS16BE(0xAA)
	if err != nil {
		return err
	}
	this.ac2, err = this.i2c.ReadRegS16BE(0xAC)
	if err != nil {
		return err
	}
	this.ac3, err = this.i2c.ReadRegS16BE(0xAE)
	if err != nil {
		return err
	}
	this.ac4, err = this.i2c.ReadRegU16BE(0xB0)
	if err != nil {
		return err
	}
	this.ac5, err = this.i2c.ReadRegU16BE(0xB2)
	if err != nil {
		return err
	}
	this.ac6, err = this.i2c.ReadRegU16BE(0xB4)
	if err != nil {
		return err
	}
	this.b1, err = this.i2c.ReadRegS16BE(0xB6)
	if err != nil {
		return err
	}
	this.b2, err = this.i2c.ReadRegS16BE(0xB8)
	if err != nil {
		return err
	}
	this.mb, err = this.i2c.ReadRegS16BE(0xBA)
	if err != nil {
		return err
	}
	this.mc, err = this.i2c.ReadRegS16BE(0xBC)
	if err != nil {
		return err
	}
	this.md, err = this.i2c.ReadRegS16BE(0xBE)
	if err != nil {
		return err
	}
	return nil
}

func (this *BMP) ReadUncompTemp() (int, error) {
	err := this.i2c.WriteRegU8(0xF4, 0x2E)
	if err != nil {
		return 0, err
	}
	time.Sleep(5 * time.Millisecond)
	const READ_REG = 0xF6
	w, err := this.i2c.ReadRegU16BE(READ_REG)
	if err != nil {
		return 0, err
	}
	return int(w), nil
}

func (this *BMP) ReadTemperature() (float32, error) {
	ut, err := this.ReadUncompTemp()
	if err != nil {
		return 0, err
	}
	err = this.readCoefficients()
	if err != nil {
		return 0, err
	}
	// Calculate temperature according to BMP180 specification
	x1 := ((ut - int(this.ac6)) * int(this.ac5)) >> 15
	log.Debug("x1=%v", x1)
	x2 := (int(this.mc) << 11) / (x1 + int(this.md))
	log.Debug("x2=%v", x2)
	b5 := x1 + x2
	log.Debug("b5=%v", b5)
	t := float32((b5+8)>>4) / 10
	log.Debug("t=%v", t)
	return t, nil
}

func (this *BMP) ReadUncompPressure(mode AccuracyMode) (int, error) {
	err := this.i2c.WriteRegU8(0xF4, 0x34+(byte(mode)<<6))
	if err != nil {
		return 0, err
	}
	if mode == AM_ULTRA_LOW_POWER {
		time.Sleep(5 * time.Millisecond)
	} else if mode == AM_STANDARD {
		time.Sleep(8 * time.Millisecond)
	} else if mode == AM_HIGH_RESOLUTION {
		time.Sleep(14 * time.Millisecond)
	} else {
		time.Sleep(26 * time.Millisecond)
	}
	const READ_REG = 0xF6
	msb, err := this.i2c.ReadRegU8(READ_REG)
	if err != nil {
		return 0, err
	}
	lsb, err := this.i2c.ReadRegU8(READ_REG + 1)
	if err != nil {
		return 0, err
	}
	xlsb, err := this.i2c.ReadRegU8(READ_REG + 2)
	if err != nil {
		return 0, err
	}
	up := (int(msb)<<16 + int(lsb)<<8 + int(xlsb)) >> (8 - uint(mode))
	return up, nil
}

func (this *BMP) ReadPressure(mode AccuracyMode) (float32, error) {
	ut, err := this.ReadUncompTemp()
	if err != nil {
		return 0, err
	}
	log.Debug("ut=%v", ut)
	up, err := this.ReadUncompPressure(mode)
	if err != nil {
		return 0, err
	}
	log.Debug("up=%v", up)
	err = this.readCoefficients()
	if err != nil {
		return 0, err
	}
	// Calculate pressure according to BMP180 specification
	x1 := ((ut - int(this.ac6)) * int(this.ac5)) >> 15
	log.Debug("x1=%v", x1)
	x2 := (int(this.mc) << 11) / (x1 + int(this.md))
	log.Debug("x2=%v", x2)
	b5 := x1 + x2
	log.Debug("b5=%v", b5)
	b6 := b5 - 4000
	log.Debug("b6=%v", b6)
	x1 = (int(this.b2) * ((b6 * b6) >> 12)) >> 11
	log.Debug("x1=%v", x1)
	x2 = (int(this.ac2) * b6) >> 11
	log.Debug("x2=%v", x2)
	x3 := x1 + x2
	log.Debug("x3=%v", x3)
	b3 := (((int(this.ac1)*4 + x3) << uint(mode)) + 2) / 4
	log.Debug("b3=%v", b3)
	x1 = (int(this.ac3) * b6) >> 13
	log.Debug("x1=%v", x1)
	x2 = ((int(this.b1) * (b6 * b6)) >> 12) >> 16
	log.Debug("x2=%v", x2)
	x3 = ((x1 + x2) + 2) >> 2
	log.Debug("x3=%v", x3)
	b4 := (uint(this.ac4) * uint(x3+32768)) >> 15
	log.Debug("b4=%v", b4)
	b7 := (uint(up) - uint(b3)) * (50000 >> uint(mode))
	log.Debug("b7=%v", b7)
	var p int
	if b7 < 0x80000000 {
		p = int((b7 * 2) / b4)
	} else {
		p = int((b7 / b4) * 2)
	}
	log.Debug("p=%v", p)
	x1 = (p >> 8) * (p >> 8)
	log.Debug("x1=%v", x1)
	x1 = (x1 * 3038) >> 16
	log.Debug("x1=%v", x1)
	x2 = (-7357 * p) >> 16
	log.Debug("x2=%v", x2)
	p = p + (x1+x2+3791)>>4
	log.Debug("p=%v", p)

	return 0, nil
}
