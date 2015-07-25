package bmp

import (
	"fmt"
	"math"
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

// Accuracy mode for calculation of atmospheric pressure.
// Affect to value accuracy, calculation time frame and power consumption.
type AccuracyMode int

const (
	AM_ULTRA_LOW_POWER AccuracyMode = iota
	AM_STANDARD
	AM_HIGH_RESOLUTION
	AM_ULTRA_HIGH_RESOLUTION
)

// Struct to keep BMP sensor data.
type BMP struct {
	sensorType SensorType
	i2c        *i2c.I2C
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
	// Sensor id
	id uint8
}

// Used registers.
const (
	COEF_AC1_REG = 0xAA
	COEF_AC2_REG = 0xAC
	COEF_AC3_REG = 0xAE
	COEF_AC4_REG = 0xB0
	COEF_AC5_REG = 0xB2
	COEF_AC6_REG = 0xB4
	COEF_B1_REG  = 0xB6
	COEF_B2_REG  = 0xB8
	COEF_MB_REG  = 0xBA
	COEF_MC_REG  = 0xBC
	COEF_MD_REG  = 0xBE
	ID_REG       = 0xD0
	CMD_REG      = 0xF4
	READ_REG     = 0xF6
)

// Create new sensor object.
func NewBMP(sensorType SensorType, i2c *i2c.I2C) (*BMP, error) {
	this := &BMP{sensorType: sensorType, i2c: i2c}
	err := this.readCoefficients()
	if err != nil {
		return nil, err
	}
	return this, nil
}

// Read compensation coefficients, which unique for each sensor.
func (this *BMP) readCoefficients() error {
	var err error
	this.ac1, err = this.i2c.ReadRegS16BE(COEF_AC1_REG)
	if err != nil {
		return err
	}
	this.ac2, err = this.i2c.ReadRegS16BE(COEF_AC2_REG)
	if err != nil {
		return err
	}
	this.ac3, err = this.i2c.ReadRegS16BE(COEF_AC3_REG)
	if err != nil {
		return err
	}
	this.ac4, err = this.i2c.ReadRegU16BE(COEF_AC4_REG)
	if err != nil {
		return err
	}
	this.ac5, err = this.i2c.ReadRegU16BE(COEF_AC5_REG)
	if err != nil {
		return err
	}
	this.ac6, err = this.i2c.ReadRegU16BE(COEF_AC6_REG)
	if err != nil {
		return err
	}
	this.b1, err = this.i2c.ReadRegS16BE(COEF_B1_REG)
	if err != nil {
		return err
	}
	this.b2, err = this.i2c.ReadRegS16BE(COEF_B2_REG)
	if err != nil {
		return err
	}
	this.mb, err = this.i2c.ReadRegS16BE(COEF_MB_REG)
	if err != nil {
		return err
	}
	this.mc, err = this.i2c.ReadRegS16BE(COEF_MC_REG)
	if err != nil {
		return err
	}
	this.md, err = this.i2c.ReadRegS16BE(COEF_MD_REG)
	if err != nil {
		return err
	}
	return nil
}

func checkCoefficient(coef uint16, name string) error {
	if coef == 0 || coef == 0xFFFF {
		return fmt.Errorf("Coefficient %s is invalid: 0x%X", name, coef)
	}
	return nil
}

func (this *BMP) IsValidCoefficients() error {
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

// Read compensation coefficients, which unique for each sensor.
func (this *BMP) readSensorID() error {
	var err error
	this.id, err = this.i2c.ReadRegU8(ID_REG)
	if err != nil {
		return err
	}
	return nil
}

func (this *BMP) IsValidSensorID() error {
	var validId byte = 0x55
	if this.id != validId {
		return fmt.Errorf("Sensor id should be 0x%X, but 0x%X received",
			validId, this.id)
	}
	return nil
}

// Read register 0xF4 for "busy" flag, according to sensor specification.
func (this *BMP) isBusy() (busy bool, err error) {
	// Check flag to know status of calculation, according
	// to specification about SCO (Start of conversion) flag
	b, err := this.i2c.ReadRegU8(CMD_REG)
	if err != nil {
		return false, err
	}
	b = b & 0x20
	log.Debug("Busy flag=0x%0X", b)
	return b != 0, nil
}

// Wait until sensor completes measurements and calculations,
// otherwise return on timeout.
func (this *BMP) waitFotCompletion() (timeout bool, err error) {
	for i := 0; i < 10; i++ {
		flag, err := this.isBusy()
		if err != nil {
			return false, err
		}
		if flag == false {
			return false, nil
		}
		time.Sleep(5 * time.Millisecond)
	}
	return true, nil
}

// Read uncompensated temprature from sensor.
func (this *BMP) ReadUncompTemp() (int, error) {
	err := this.i2c.WriteRegU8(CMD_REG, 0x2E)
	if err != nil {
		return 0, err
	}
	_, err = this.waitFotCompletion()
	if err != nil {
		return 0, err
	}
	w, err := this.i2c.ReadRegU16BE(READ_REG)
	if err != nil {
		return 0, err
	}
	return int(w), nil
}

// Read and calculate temrature in C (celsius).
func (this *BMP) ReadTemperature() (float32, error) {
	ut, err := this.ReadUncompTemp()
	if err != nil {
		return 0, err
	}
	err = this.readCoefficients()
	if err != nil {
		return 0, err
	}
	// Calculate temperature according to sensor specification
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

// Read atmospheric uncompensated pressure from sensor.
func (this *BMP) ReadUncompPressure(mode AccuracyMode) (int, error) {
	err := this.i2c.WriteRegU8(CMD_REG, 0x34+(byte(mode)<<6))
	if err != nil {
		return 0, err
	}
	_, err = this.waitFotCompletion()
	if err != nil {
		return 0, err
	}
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

// Read and calculate atmospheric pressure in Pa (Pascal).
func (this *BMP) ReadPressurePa(mode AccuracyMode) (int, error) {
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
	// Calculate pressure according to sensor specification
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
	return p, nil
}

// Read and calculate atmospheric pressure in mmHg (millimeter of mercury).
func (this *BMP) ReadPressureMmHg(mode AccuracyMode) (float32, error) {
	p, err := this.ReadPressurePa(mode)
	if err != nil {
		return 0, err
	}
	// Amount of Pa in 1 mmHg
	var mmHg float32 = 133.322
	// Round up to 2 decimals after point
	p2 := float32(int(float32(p)/mmHg*100)) / 100
	return p2, nil
}

// Read and calculate altitude above sea level, if we assume
// that pressure at see level is equal to 101325 Pa.
func (this *BMP) ReadAltitude(mode AccuracyMode) (float32, error) {
	p, err := this.ReadPressurePa(mode)
	if err != nil {
		return 0, err
	}
	// Approximate atmospheric pressure at sea level in Pa
	p0 := 101325.0
	a := 44330 * (1 - math.Pow(float64(p)/p0, 1/5.255))
	// Round up to 2 decimals after point
	a2 := float32(int(a*100)) / 100
	return a2, nil
}
