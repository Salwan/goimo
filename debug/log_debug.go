//go:build !release

package debug

import "fmt"

func Log(topic, msg string, args ...interface{}) {
	if len(args) > 0 {
		msg = fmt.Sprintf(msg, args...)
	}
	fmt.Printf("[%s] %s\n", topic, msg)
}

func GjkLog(msg string, args ...interface{}) {
	Log("GjkEpa", msg, args...)
}
