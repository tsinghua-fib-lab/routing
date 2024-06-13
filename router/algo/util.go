package algo

func TimeToIndex(time float64) int {
	// 输入time返回对应时间片下标
	index := int(time / TIME_SLICE_INTERVAl)
	if index > TIME_SLICE_LENGTH-1 {
		index = TIME_SLICE_LENGTH - 1
	}
	return index
}
