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
	"fmt"
	"time"

	i2c "github.com/d2r2/go-i2c"
)

// Utility functions

// getS16BE extract 2-byte integer as signed big-endian.
func getS16BE(buf []byte) int16 {
	v := int16(buf[0])<<8 + int16(buf[1])
	return v
}

// getS16LE extract 2-byte integer as signed little-endian.
func getS16LE(buf []byte) int16 {
	w := getS16BE(buf)
	// exchange bytes
	v := (w&0xFF)<<8 + w>>8
	return v
}

// getU16BE extract 2-byte integer as unsigned big-endian.
func getU16BE(buf []byte) uint16 {
	v := uint16(buf[0])<<8 + uint16(buf[1])
	return v
}

// getU16LE extract 2-byte integer as unsigned little-endian.
func getU16LE(buf []byte) uint16 {
	w := getU16BE(buf)
	// exchange bytes
	v := (w&0xFF)<<8 + w>>8
	return v
}

// checkCoefficient verify that compensation parameter looks valid.
func checkCoefficient(coef uint16, name string) error {
	if coef == 0 || coef == 0xFFFF {
		return fmt.Errorf("Coefficient %s is invalid: 0x%X", name, coef)
	}
	return nil
}

// waitForCompletion Wait until sensor completes measurements and calculations,
// otherwise return on timeout.
func waitForCompletion(sensor SensorInterface, i2c *i2c.I2C) (timeout bool, err error) {
	for i := 0; i < 10; i++ {
		flag, err := sensor.IsBusy(i2c)
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

// Read byte block from i2c device to struct object.
func readDataToStruct(i2c *i2c.I2C, byteCount int,
	byteOrder binary.ByteOrder, obj interface{}) error {
	buf1 := make([]byte, byteCount)
	_, err := i2c.ReadBytes(buf1)
	if err != nil {
		return err
	}
	buf := bytes.NewBuffer(buf1)
	err = binary.Read(buf, byteOrder, obj)
	if err != nil {
		return err
	}
	return nil
}
