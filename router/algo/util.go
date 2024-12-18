package algo

import (
	"math"

	"git.fiblab.net/general/common/v2/geometry"
)

func TimeToIndex(time float64) int {
	// 输入time返回对应时间片下标
	index := int(time / TIME_SLICE_INTERVAl)
	if index > TIME_SLICE_LENGTH-1 {
		index = TIME_SLICE_LENGTH - 1
	} else if index < 0 {
		index = 0
	}
	return index
}
func PointToTaz(p geometry.Point, xStep float64, yStep float64, xMin float64, yMin float64) TazPair {
	return TazPair{
		X: int32((p.X - xMin) / xStep),
		Y: int32((p.Y - yMin) / yStep),
	}
}
func PointToNearTAZs(p geometry.Point, xStep float64, yStep float64, xMin float64, yMin float64) []TazPair {
	centerX, centerY := int32((p.X-xMin)/xStep), int32((p.Y-yMin)/yStep)
	tazs := make([]TazPair, 0)
	// 添加中心坐标
	tazs = append(tazs, TazPair{X: centerX, Y: centerY})
	for _, deltaX := range []int32{-1, 0, 1} {
		for _, deltaY := range []int32{-1, 0, 1} {
			if deltaX == 0 && deltaY == 0 {
				continue
			} else {
				tazs = append(tazs, TazPair{X: centerX + deltaX, Y: centerY + deltaY})
			}

		}

	}
	return tazs
}
func TazDistance(taz1 TazPair, taz2 TazPair, xStep float64, yStep float64) float64 {
	disX := xStep * math.Abs(float64(taz1.X)-float64(taz2.X))
	disY := yStep * math.Abs(float64(taz1.Y)-float64(taz2.Y))
	return math.Hypot(disX, disY)
}
func InServiceTime(cost TazCost, time float64) bool {
	return time >= cost.SublineStartTime && time <= cost.SublineEndTime
}
func TazCenterPoint(taz TazPair, xStep float64, yStep float64, xMin float64, yMin float64) geometry.Point {
	return geometry.Point{
		X: (float64(taz.X)+0.5)*xStep + xMin,
		Y: (float64(taz.Y)+0.5)*yStep + yMin,
	}
}
func StationTimeDependentCosts(cost float64, departureTimes []float64) []float64 {
	costs := make([]float64, TIME_SLICE_LENGTH)
	for index := range costs {
		costs[index] = math.Inf(0)
	}
	if len(departureTimes) > 0 {
		startIndex := TimeToIndex(departureTimes[0])
		endIndex := TimeToIndex(departureTimes[len(departureTimes)-1])
		for index := range costs {
			if index >= startIndex && index <= endIndex {
				costs[index] = cost
			}

		}
	}
	return costs

}
