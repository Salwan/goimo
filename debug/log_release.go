//go:build release

package debug

func Log(topic, msg string, args ...interface{}) {}
func GjkLog(msg string, args ...interface{})     {}
